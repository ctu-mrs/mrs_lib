#include "mrs_lib/timer.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>
#include <utility>

#include <rclcpp/clock.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>

#include "mrs_lib/internal/coroutine_callback_helpers.hpp"
#include "mrs_lib/utility/owning_mutex.hpp"

namespace mrs_lib
{

  namespace
  {

    class RosTimerImpl : public internal::TimerImplInterface
    {
      struct State
      {
        bool is_first_;
        std::shared_ptr<rclcpp::TimerBase> timer_;
      };

    public:
      RosTimerImpl(RosTimerOptions options, std::chrono::nanoseconds period, Timer::EventCallback callback)
          : node_interfaces_(options.node_interfaces),
            callback_group_(options.callback_group),
            is_oneshot_(options.oneshot),
            user_callback_(std::move(callback)),
            state_(State{
                .is_first_ = true,
                .timer_ = create_timer(period, false),
            })
      {
        // The timer is always initialized paused to prevent a potential
        // (although unlikely) race condition. If it was started immediately, it
        // could trigger befor the construction of the state_ member was
        // completed and thus cause UB.
        if (options.autostart)
        {
          auto guard = state_.acquire();
          guard->timer_->reset();
        }
      }

      void stop() override
      {
        auto guard = state_.acquire();
        guard->timer_->cancel();
      }

      void start_or_reset() override
      {
        auto guard = state_.acquire();
        guard->is_first_ = true;
        guard->timer_->reset();
      }

      void set_period(std::chrono::nanoseconds period) override
      {
        auto guard = state_.acquire();
        guard->is_first_ = true;
        guard->timer_ = create_timer(period, true);
      }

      bool is_running() const override
      {
        auto guard = state_.acquire();
        return !guard->timer_->is_canceled();
      }

    private:
      std::shared_ptr<rclcpp::TimerBase> create_timer(std::chrono::nanoseconds period, bool start)
      {
        std::function<void()> callback = std::bind_front(&RosTimerImpl::timer_callback, this);
        return rclcpp::create_timer(node_interfaces_, node_interfaces_.get_node_clock_interface()->get_clock(), rclcpp::Duration(period), std::move(callback),
                                    callback_group_, start);
      }

      void timer_callback()
      {
        if (is_oneshot_)
        {
          auto guard = state_.acquire();
          // If is_first_ was already false, another callback was faster so we need
          // to return from this one.
          if (!std::exchange(guard->is_first_, false))
          {
            return;
          }
          // Since the timer is oneshot, we need to cancel it now.
          guard->timer_->cancel();
        }

        user_callback_();
      }

      TimerNodeInterfaces node_interfaces_;
      std::shared_ptr<rclcpp::CallbackGroup> callback_group_;
      bool is_oneshot_;
      std::function<void()> user_callback_;

      OwningMutex<State> state_;
    };

    class ThreadTimerImpl : public internal::TimerImplInterface
    {
      using Clock = std::chrono::steady_clock;

    public:
      ThreadTimerImpl(ThreadTimerOptions options, std::chrono::nanoseconds period, Timer::EventCallback callback)
          : context_(options.node_interfaces.get_node_base_interface()->get_context()),
            is_oneshot_(options.oneshot),
            user_callback_(std::move(callback)),
            is_running_(options.autostart),
            period_(period),
            next_expected_(Clock::now() + period),
            thread_(std::bind_front(&ThreadTimerImpl::thread_function, this))
      {
      }

      void stop() override
      {
        std::lock_guard lock(sleep_data_mutex_);
        is_running_ = false;
        sleep_cv_.notify_all();
      }

      void start_or_reset() override
      {
        std::lock_guard lock(sleep_data_mutex_);
        next_expected_ = Clock::now() + period_;
        is_running_ = true;
        sleep_cv_.notify_all();
      }

      void set_period(std::chrono::nanoseconds period) override
      {
        std::lock_guard lock(sleep_data_mutex_);
        period_ = period;
        next_expected_ = Clock::now() + period_;
        is_running_ = true;
        sleep_cv_.notify_all();
      }

      [[nodiscard]] bool is_running() const override
      {
        std::lock_guard lock(sleep_data_mutex_);
        return is_running_;
      }

    private:
      bool wait_for_start(std::unique_lock<std::mutex>& lock, std::stop_token token)
      {
        return sleep_cv_.wait(lock, token, [this]() { return is_running_; });
      }

      bool breakable_sleep(std::unique_lock<std::mutex>& lock, std::stop_token token)
      {
        auto should_cancel = [&, this]() { return !rclcpp::ok(context_) || !is_running_ || token.stop_requested(); };

        // The next_expected_ member might change while we sleep by calls to reset.
        // Therefore, we may need to sleep here multiple times, or return if canceled.
        while (next_expected_ > Clock::now())
        {
          sleep_cv_.wait_until(lock, token, next_expected_, should_cancel);

          if (should_cancel())
          {
            return false;
          }
        }

        return true;
      }

      void thread_function(std::stop_token token)
      {
        while (rclcpp::ok(context_) && !token.stop_requested())
        {
          {
            std::unique_lock lock(sleep_data_mutex_);

            bool wait_success = wait_for_start(lock, token);
            if (!wait_success)
            {
              continue;
            }

            const bool sleep_success = breakable_sleep(lock, token);
            if (!sleep_success)
            {
              continue;
            }

            auto now = Clock::now();
            next_expected_ = std::max(next_expected_ + period_, now);
            if (is_oneshot_)
            {
              is_running_ = false;
            }
          }

          // Run the user callback after ulocking the mutex.
          user_callback_();
        }
      }

      std::shared_ptr<rclcpp::Context> context_;
      bool is_oneshot_;
      std::function<void()> user_callback_;

      mutable std::mutex sleep_data_mutex_{};
      std::condition_variable_any sleep_cv_{};

      bool is_running_;
      std::chrono::nanoseconds period_;
      Clock::time_point next_expected_;

      // Thread is last member to be constructed last and destroyed first.
      std::jthread thread_;
    };

  } // namespace

  Timer::Timer(const RosTimerOptions& options, std::chrono::nanoseconds period, EventCallback callback)
      : impl_(std::in_place_type_t<RosTimerImpl>{}, options, period, std::move(callback))
  {
  }

  Timer::Timer(const RosTimerOptions& options, std::chrono::nanoseconds period, EventCoroCallback callback)
      : Timer(options, period, internal::get_detached_coro_callback_launcher(std::move(callback), options.callback_group))
  {
  }

  Timer::Timer(const ThreadTimerOptions& options, std::chrono::nanoseconds period, EventCallback callback)
      : impl_(std::in_place_type_t<ThreadTimerImpl>{}, options, period, std::move(callback))
  {
  }

  Timer::Timer(const ThreadTimerOptions& options, std::chrono::nanoseconds period, EventCoroCallback callback)
      : Timer(options, period, internal::get_detached_coro_callback_launcher(std::move(callback), internal::NoCallbackGroupTag{}))
  {
  }

  void Timer::stop()
  {
    impl_->stop();
  }

  void Timer::start_or_reset()
  {
    impl_->start_or_reset();
  }

  void Timer::set_period(std::chrono::nanoseconds period)
  {
    impl_->set_period(period);
  }

  bool Timer::is_running() const
  {
    return impl_->is_running();
  }

} // namespace mrs_lib
