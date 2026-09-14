#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/time.hpp>

#include "mrs_lib/timeout_manager.hpp"
#include "mrs_lib/timer.hpp"

namespace mrs_lib
{
  /** @private */
  class TimeoutManager2::Impl
  {
  public:
    Impl(TimeoutManagerNodeInterfaces node_interfaces, std::chrono::nanoseconds update_period)
        : node_interfaces_(std::move(node_interfaces)),
          main_timer_(RosTimerOptions{.node_interfaces = node_interfaces_}, update_period, std::bind_front(&Impl::main_timer_callback, this))
    {
    }

    ~Impl() = default;

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    std::shared_ptr<TimeoutManagerHandle> register_new(TimeoutManagerRegisterOptions options)
    {
      auto handle =
          std::make_shared<TimeoutManagerHandle>(internal::TimeoutManagerHandleKey{}, std::move(options), node_interfaces_.get_node_clock_interface());
      auto timeouts_guard = timeouts_.acquire();
      timeouts_guard->emplace_back(handle);
      return handle;
    }

    void start_all()
    {
      auto timeouts_guard = timeouts_.acquire();
      clear_deleted_impl(*timeouts_guard);

      for (auto&& weak_ptr : *timeouts_guard)
      {
        auto timeout_info = weak_ptr.lock();
        // Some may have expired after the clear, so we need to check again.
        if (timeout_info == nullptr)
        {
          continue;
        }

        timeout_info->start();
      }
    }

    void pause_all()
    {
      auto timeouts_guard = timeouts_.acquire();
      clear_deleted_impl(*timeouts_guard);

      for (auto&& weak_ptr : *timeouts_guard)
      {
        auto timeout_info = weak_ptr.lock();
        // Some may have expired after the clear, so we need to check again.
        if (timeout_info == nullptr)
        {
          continue;
        }

        timeout_info->pause();
      }
    }

    void clear_deleted()
    {
      auto timeouts_guard = timeouts_.acquire();
      clear_deleted_impl(*timeouts_guard);
    }

  private:
    static void clear_deleted_impl(std::vector<std::weak_ptr<TimeoutManagerHandle>>& timeouts)
    {
      std::erase_if(timeouts, [](const std::weak_ptr<TimeoutManagerHandle>& weak_ptr) { return weak_ptr.expired(); });
    }


    void main_timer_callback()
    {
      auto timeouts_guard = timeouts_.acquire();

      for (auto&& weak_ptr : *timeouts_guard)
      {
        auto timeout_info = weak_ptr.lock();
        if (timeout_info == nullptr)
        {
          continue;
        }

        timeout_info->run_callback_if_expired(internal::TimeoutManagerHandleKey{});
      }
    }

    TimeoutManagerNodeInterfaces node_interfaces_;

    OwningMutex<std::vector<std::weak_ptr<TimeoutManagerHandle>>> timeouts_{};
    Timer main_timer_;
  };


  TimeoutManager2::TimeoutManager2(TimeoutManagerNodeInterfaces node_interfaces, std::chrono::nanoseconds update_period)
      : impl_(std::in_place_type_t<Impl>{}, std::move(node_interfaces), update_period)
  {
  }

  TimeoutManager2::~TimeoutManager2() = default;

  TimeoutManager2::TimeoutManager2(TimeoutManager2&&) noexcept = default;

  TimeoutManager2& TimeoutManager2::operator=(TimeoutManager2&&) noexcept = default;

  auto TimeoutManager2::register_new(TimeoutManagerRegisterOptions options) -> std::shared_ptr<TimeoutManagerHandle>
  {
    return impl_->register_new(std::move(options));
  }

  void TimeoutManager2::pause_all()
  {
    impl_->pause_all();
  }

  void TimeoutManager2::start_all()
  {
    impl_->start_all();
  }

  void TimeoutManager2::clear_deleted()
  {
    impl_->clear_deleted();
  }

  TimeoutManagerHandle::TimeoutManagerHandle(internal::TimeoutManagerHandleKey, TimeoutManagerRegisterOptions options,
                                             std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface)
      : data_(std::invoke([&]() -> Data {
          auto now = clock_interface->get_clock()->now();
          if (!options.callback)
          {
            throw std::logic_error("TimeoutManagerRegisterOptions must contain a valid callback.");
          }
          return {
              .clock_interface = std::move(clock_interface),
              .options = std::move(options),
              .last_reset = now,
              .last_callback = now,
          };
        }))
  {
  }

  void TimeoutManagerHandle::reset_timeout()
  {
    auto guard = data_.acquire();
    guard->last_reset = guard->clock_interface->get_clock()->now();
  }

  void TimeoutManagerHandle::start()
  {
    auto guard = data_.acquire();
    guard->options.started = true;
    guard->last_reset = guard->clock_interface->get_clock()->now();
  }

  void TimeoutManagerHandle::pause()
  {
    auto guard = data_.acquire();
    guard->options.started = false;
  }

  void TimeoutManagerHandle::set_timeout(std::chrono::nanoseconds timeout)
  {
    auto guard = data_.acquire();
    guard->options.timeout = timeout;
  }

  bool TimeoutManagerHandle::is_started() const
  {
    auto guard = data_.acquire();
    return guard->options.started;
  }

  rclcpp::Time TimeoutManagerHandle::get_last_reset() const
  {
    auto guard = data_.acquire();
    return guard->last_reset;
  }

  void TimeoutManagerHandle::run_callback_if_expired(internal::TimeoutManagerHandleKey)
  {
    auto guard = data_.acquire();
    const rclcpp::Time now = guard->clock_interface->get_clock()->now();

    const bool started = guard->options.started;
    const bool last_reset_timeout_expired = (now - guard->last_reset) >= guard->options.timeout;
    const bool last_callback_timeout_expired = (now - guard->last_callback) >= guard->options.timeout;

    if (started && last_reset_timeout_expired && last_callback_timeout_expired)
    {
      guard->options.callback(guard->last_reset);
      guard->last_callback = now;

      // if the timeout is oneshot, pause it
      if (guard->options.oneshot)
      {
        guard->options.started = false;
      }
    }
  }

} // namespace mrs_lib
