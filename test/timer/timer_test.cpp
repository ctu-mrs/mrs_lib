#include "mrs_lib/timer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <format>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <variant>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "mrs_lib/logger.hpp"
#include "mrs_lib/utility/callback.hpp"

#include "mrs_lib_testing/ros_fixtures.hpp"


namespace
{
  namespace example
  {

    // DOCS: BEGIN EXAMPLE
    using namespace std::chrono_literals;

    class ExampleNode : public rclcpp::Node
    {
    public:
      ExampleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
          : Node("example_node", options),
            logger_(*this),
            timer_(
                mrs_lib::RosTimerOptions{
                    .node_interfaces = *this,
                    // You can specify other options here, but the defaults
                    // are good for us.
                },
                100ms, [this]() { timer_callback(); })
      {
      }

      [[nodiscard]] size_t get_callbacks_count() const
      {
        return callbacks_count_;
      }

    private:
      void timer_callback()
      {
        logger_.info("Timer callback!");
        callbacks_count_ += 1;
      }

      mrs_lib::Logger logger_;

      size_t callbacks_count_ = 0;

      mrs_lib::Timer timer_;
    };
    // DOCS: END EXAMPLE


    class TimerExampleTest : public mrs_lib_testing::RosExecutorFixture<>
    {
    };

    TEST_F(TimerExampleTest, Example)
    {
      auto node = std::make_shared<ExampleNode>();
      get_executor().add_node(node);

      std::this_thread::sleep_for(550ms);
      const size_t callbacks_count = node->get_callbacks_count();
      const size_t expected_callbacks_count = 5;

      EXPECT_EQ(callbacks_count, expected_callbacks_count);

      get_executor().remove_node(node);
    }


  } // namespace example

  using namespace std::chrono_literals;

  constexpr int test_repetitions_count = 8;

  enum class TestTimerType
  {
    ros,
    thread
  };

} // namespace

template <>
struct std::formatter<TestTimerType, char>
{

  template <class ParseContext>
  constexpr ParseContext::iterator parse(ParseContext& ctx)
  {
    return ctx.begin();
  }

  template <class FmtContext>
  FmtContext::iterator format(TestTimerType timer_type, FmtContext& ctx) const
  {
    switch (timer_type)
    {
    case TestTimerType::ros:
      return std::format_to(ctx.out(), "ros");
    case TestTimerType::thread:
      return std::format_to(ctx.out(), "thread");
    }
    return std::format_to(ctx.out(), "UNKNOWN");
  }
};

namespace
{
  std::string test_timer_type_info_to_string(const ::testing::TestParamInfo<TestTimerType>& info)
  {
    return std::format("{}", info.param);
  }

  struct TimerTestCommonOptions
  {
    bool autostart = false;
    bool oneshot = false;
    bool coro_callback = false;
  };

  struct TimerTestData
  {
    TestTimerType timer_type;
    rclcpp::NodeOptions node_options;
    // double rate;
    std::chrono::milliseconds period;
    std::chrono::milliseconds callback_sleep_time;
    std::chrono::milliseconds max_drift;
  };

  std::string timer_test_data_info_to_string(const ::testing::TestParamInfo<TimerTestData>& info)
  {
    auto&& param = info.param;
    return std::format("{}_period_{}_sleep_{}", param.timer_type, param.period, param.callback_sleep_time);
  }

  class Test : public ::testing::TestWithParam<TimerTestData>
  {

  public:
    static void SetUpTestSuite()
    {
      std::cout << "Ros initialized.\n" << std::flush;
      rclcpp::init(0, nullptr);
    }


    static void TearDownTestSuite()
    {
      rclcpp::shutdown();
      std::cout << "Ros shut down.\n" << std::flush;
    }

  protected:
    void SetUp() override
    {
      const rclcpp::NodeOptions& node_options = GetParam().node_options;
      period_ = GetParam().period;
      // rate_ = GetParam().rate;
      callback_sleep_time_ = GetParam().callback_sleep_time;
      max_drift_ = GetParam().max_drift;

      node_ = std::make_shared<rclcpp::Node>("test_timer", node_options);
      reentrant_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);

      finished_future_ = finished_promise_.get_future();

      main_thread_ = std::thread(&Test::spin, this);
      while (!executor_->is_spinning())
      {
        std::cout << "Waiting for executor to start...\n" << std::flush;
        std::this_thread::sleep_for(1ms);
      }
    }

    void spin()
    {

      RCLCPP_INFO(node_->get_logger(), "starting spinning");

      executor_->spin();

      RCLCPP_INFO(node_->get_logger(), "stopped spinning");
    }

    void TearDown() override
    {
      executor_->cancel();

      main_thread_.join();
      RCLCPP_INFO(node_->get_logger(), "thread joined");
    }

    void initializeTimer(TimerTestCommonOptions timer_opts)
    {
      using CallbackVariant = std::variant<mrs_lib::Timer::EventCallback, mrs_lib::Timer::EventCoroCallback>;
      using OptionsVariant = std::variant<mrs_lib::RosTimerOptions, mrs_lib::ThreadTimerOptions>;

      RCLCPP_INFO(node_->get_logger(), "Constructing timer");

      auto callback_variant = std::invoke([&]() -> CallbackVariant {
        if (timer_opts.coro_callback)
        {
          return mrs_lib::CoroCallback<void()>(mrs_lib::coro_callback_tags::CancelNewDefault{}, &Test::timerCoroCallback, this);
        } else
        {
          return std::function<void()>([this]() { this->timerCallback(); });
        }
      });

      auto options_variant = std::invoke([&]() -> OptionsVariant {
        switch (GetParam().timer_type)
        {
        case TestTimerType::ros:
          return mrs_lib::RosTimerOptions{
              .node_interfaces = *node_,
              .autostart = timer_opts.autostart,
              .oneshot = timer_opts.oneshot,
              // Coroutine callbacks require reentrant callback group.
              .callback_group = (timer_opts.coro_callback) ? reentrant_callback_group_ : nullptr,
          };
        case TestTimerType::thread:
          return mrs_lib::ThreadTimerOptions{
              .node_interfaces = *node_,
              .autostart = timer_opts.autostart,
              .oneshot = timer_opts.oneshot,
          };
        }
        throw std::logic_error("Unhandled timer type");
      });

      timer_ = std::visit(
          [this](auto&& options, auto&& callback) {
            return std::make_shared<mrs_lib::Timer>(std::forward<decltype(options)>(options), period_, std::forward<decltype(callback)>(callback));
          },
          options_variant, callback_variant);
    }

    void destroyTimer()
    {
      RCLCPP_INFO(node_->get_logger(), "Destroying timer");
      timer_.reset();
    }

    void resetTestState()
    {
      n_cbks_ = 0;
      cbks_in_time_ = true;
      null_cbk_ = false;
      callback_called_after_stopped_ = false;

      timer_stop_called_from_callback_ = false;

      last_time_callback_ = std::nullopt;
    }

    void timerCallback();
    mrs_lib::Task<> timerCoroCallback()
    {
      timerCallback();
      co_return;
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::CallbackGroup> reentrant_callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

    std::thread main_thread_;

    std::promise<bool> finished_promise_;
    std::future<bool> finished_future_;


    std::chrono::nanoseconds period_;
    // double rate_;
    std::chrono::nanoseconds callback_sleep_time_;
    std::chrono::nanoseconds max_drift_;

    std::atomic<int> n_cbks_ = 0;
    bool cbks_in_time_ = true;
    bool null_cbk_ = false;
    std::atomic<bool> callback_called_after_stopped_ = false;

    std::mutex cbk_running_mtx_;
    bool timer_stop_called_from_callback_ = false;

    std::atomic<bool> test_stop_from_cbk_ = false;

    std::shared_ptr<mrs_lib::Timer> timer_;

    std::optional<rclcpp::Time> last_time_callback_;
  };

  void Test::timerCallback()
  {
    std::scoped_lock lck(cbk_running_mtx_);

    n_cbks_++;

    const auto now = node_->get_clock()->now();

    if (last_time_callback_)
    {

      rclcpp::Time expected_time = last_time_callback_.value() + rclcpp::Duration(std::chrono::duration<double>(period_));
      rclcpp::Duration dt = now - expected_time;
      last_time_callback_.value() = expected_time;

      double max_dt = std::chrono::duration<double>(max_drift_).count();

      if (std::abs(dt.seconds()) > max_dt)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback did not come in time! Expected: " << expected_time.seconds() << ", received: " << now.seconds()
                                                                                             << " (period: " << period_ << ", difference: " << dt.seconds()
                                                                                             << ")");
        cbks_in_time_ = false;
        RCLCPP_INFO(node_->get_logger(), "Resetting drift.");
        last_time_callback_ = now;
      }

    } else
    {
      RCLCPP_INFO(node_->get_logger(), "Skipping first iter");
      last_time_callback_ = now;
    }

    if (!timer_)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback called while timer is already destroyed!");
      null_cbk_ = true;
      return;
    }

    if (timer_stop_called_from_callback_)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback called while timer is not running!");
      callback_called_after_stopped_ = true;
    }

    if (test_stop_from_cbk_)
    {
      timer_->stop();
      std::cout << "\tStopping timer from callback.\n" << std::flush;
      // Mark the timer stopped to test that there are no further callback
      timer_stop_called_from_callback_ = true;
    }

    rclcpp::sleep_for(callback_sleep_time_);
  }

  TEST_P(Test, TestCallbackPeriod)
  {
    initializeTimer({});

    for (int i = 0; i < test_repetitions_count; i++)
    {
      std::cout << "\tTesting callback period\n" << std::flush;
      resetTestState();

      timer_->start_or_reset();

      const rclcpp::Time start = node_->get_clock()->now();
      const double test_dur = 1.0;

      rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * test_dur)));

      timer_->stop();
      // There may be a callback running while calling the stop.
      const int callbacks_at_stop = n_cbks_.exchange(0);
      // Wait to see if there are more callbacks after stop
      const size_t wait_periods = 4;
      rclcpp::sleep_for(wait_periods * period_);

      const double rate = 1 / std::chrono::duration<double>(period_).count();
      const double expected_cbks = test_dur * rate;
      const bool callbacks_in_time = cbks_in_time_;
      // There may have been one callback about to fire at the time of resetting the counter.
      const bool no_callbacks_after_stopped = n_cbks_ <= 1;

      EXPECT_TRUE(no_callbacks_after_stopped);
      EXPECT_LE(std::abs(callbacks_at_stop - expected_cbks), 2);
      EXPECT_TRUE(callbacks_in_time);
    }

    destroyTimer();
  }


  TEST_P(Test, TestCoroCallbackPeriod)
  {
    initializeTimer({.coro_callback = true});

    for (int i = 0; i < test_repetitions_count; i++)
    {
      std::cout << "\tTesting callback period\n" << std::flush;
      resetTestState();

      timer_->start_or_reset();

      const rclcpp::Time start = node_->get_clock()->now();
      const double test_dur = 1.0;

      rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * test_dur)));

      timer_->stop();
      // There may be a callback running while calling the stop.
      const int callbacks_at_stop = n_cbks_.exchange(0);
      // Wait to see if there are more callbacks after stop
      const size_t wait_periods = 4;
      rclcpp::sleep_for(wait_periods * period_);

      const double rate = 1 / std::chrono::duration<double>(period_).count();
      const double expected_cbks = test_dur * rate;
      const bool callbacks_in_time = cbks_in_time_;
      // There may have been one callback about to fire at the time of resetting the counter.
      const bool no_callbacks_after_stopped = n_cbks_ <= 1;

      EXPECT_TRUE(no_callbacks_after_stopped);
      EXPECT_LE(std::abs(callbacks_at_stop - expected_cbks), 2);
      EXPECT_TRUE(callbacks_in_time);
    }

    destroyTimer();
  }

  TEST_P(Test, StopFromCallback)
  {
    initializeTimer({});
    test_stop_from_cbk_ = true;

    for (int i = 0; i < test_repetitions_count; i++)
    {
      std::cout << "\tTesting stop from callback\n" << std::flush;
      resetTestState();
      timer_->start_or_reset();

      // wait for one callback to be called
      while (!n_cbks_)
      {
        rclcpp::sleep_for(1s);
      }

      // wait for the callback to end
      {
        std::scoped_lock lck(cbk_running_mtx_);
        n_cbks_ = 0;
        const bool timer_stopped_from_callback = !timer_->is_running();
        EXPECT_TRUE(timer_stopped_from_callback);
      }

      node_->get_clock()->sleep_for(2 * period_);

      EXPECT_TRUE(timer_stop_called_from_callback_);
      EXPECT_FALSE(callback_called_after_stopped_);
      const bool no_callbacks_after_stopped = n_cbks_ == 0;
      EXPECT_TRUE(no_callbacks_after_stopped);
    }

    destroyTimer();
  }

  TEST_P(Test, Destructor)
  {
    for (int i = 0; i < test_repetitions_count; i++)
    {
      initializeTimer({});
      std::cout << "\tTesting destructor\n" << std::flush;

      resetTestState();

      const rclcpp::Time start = node_->get_clock()->now();

      timer_->set_period(25ms);
      timer_->start_or_reset();
      destroyTimer();

      const rclcpp::Time destroyed = node_->get_clock()->now();
      const bool callback_while_destroyed = null_cbk_;
      // There may have been one callback about to fire at the time of resetting the counter.
      const bool no_callbacks_after_stopped = n_cbks_ <= 1;

      EXPECT_FALSE(callback_while_destroyed);
      EXPECT_TRUE(no_callbacks_after_stopped);
      EXPECT_LE((destroyed - start).seconds(), 1.0);
    }
  }

  TEST_P(Test, Oneshot)
  {
    for (int i = 0; i < test_repetitions_count; i++)
    {
      std::cout << "\tTesting oneshot timer\n" << std::flush;

      const size_t wait_periods = 8;

      resetTestState();
      initializeTimer({.autostart = true, .oneshot = true});

      rclcpp::sleep_for(period_ * wait_periods);

      EXPECT_FALSE(timer_->is_running());
      EXPECT_EQ(n_cbks_, 1);
      resetTestState();

      rclcpp::sleep_for(period_ * wait_periods);

      EXPECT_FALSE(timer_->is_running());
      EXPECT_EQ(n_cbks_, 0);
      resetTestState();

      timer_->start_or_reset();

      rclcpp::sleep_for(period_ * wait_periods);

      EXPECT_FALSE(timer_->is_running());
      EXPECT_EQ(n_cbks_, 1);
      resetTestState();

      destroyTimer();
    }
  }

  INSTANTIATE_TEST_SUITE_P(RosTimerInstance, Test,
                           ::testing::Values(TimerTestData{
                               .timer_type = TestTimerType::ros,
                               .node_options = rclcpp::NodeOptions().use_intra_process_comms(false),
                               .period = 20ms,
                               .callback_sleep_time = 10ms,
                               .max_drift = 10ms,
                           }),
                           &timer_test_data_info_to_string);

  INSTANTIATE_TEST_SUITE_P(ThreadTimerInstance, Test,
                           ::testing::Values(TimerTestData{
                               .timer_type = TestTimerType::thread,
                               .node_options = rclcpp::NodeOptions().use_intra_process_comms(false),
                               .period = 20ms,
                               .callback_sleep_time = 10ms,
                               .max_drift = 10ms,
                           }),
                           &timer_test_data_info_to_string);

  class NoAutostart : public ::testing::TestWithParam<TestTimerType>
  {
  public:
    static void SetUpTestSuite()
    {
      std::cout << "Ros initialized.\n" << std::flush;
      rclcpp::init(0, nullptr);
    }


    static void TearDownTestSuite()
    {
      rclcpp::shutdown();
      std::cout << "Ros shut down.\n" << std::flush;
    }

    void SetUp() override
    {
      node_ = std::make_shared<rclcpp::Node>("test_timer_handler", rclcpp::NodeOptions{});

      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);

      spinning_thread_ = std::jthread(&NoAutostart::spin, this);

      while (!executor_->is_spinning())
      {
        std::cout << "Waiting for executor to start...\n" << std::flush;
        std::this_thread::sleep_for(1ms);
      }
    }

    void TearDown() override
    {
      executor_->cancel();

      spinning_thread_.join();
      RCLCPP_INFO(node_->get_logger(), "thread joined");
    }

    mrs_lib::Timer create_timer(std::chrono::nanoseconds period)
    {
      switch (GetParam())
      {
      case TestTimerType::ros: {
        auto opts = mrs_lib::RosTimerOptions{
            .node_interfaces = *node_,
            .autostart = false,
            .oneshot = false,
        };
        return mrs_lib::Timer(opts, period, [this]() { callback(); });
      }
      case TestTimerType::thread: {
        auto opts = mrs_lib::ThreadTimerOptions{
            .node_interfaces = *node_,
            .autostart = false,
            .oneshot = false,
        };
        return mrs_lib::Timer(opts, period, [this]() { callback(); });
      }
      }
      throw std::logic_error("Unhandled timer type");
    }

  private:
    void spin()
    {

      RCLCPP_INFO(node_->get_logger(), "starting spinning");

      executor_->spin();

      RCLCPP_INFO(node_->get_logger(), "stopped spinning");
    }


    void callback()
    {
      callbacks_count_++;
    }


    std::unique_ptr<rclcpp::Executor> executor_;
    std::jthread spinning_thread_;

  protected:
    std::shared_ptr<rclcpp::Node> node_;

    std::atomic<size_t> callbacks_count_ = 0;
  };

  TEST_P(NoAutostart, NoAutostart)
  {

    for (int i = 0; i < test_repetitions_count; i++)
    {
      std::cout << "\tTesting disabled autostart\n" << std::flush;

      const std::chrono::microseconds wait_time = 100ms;

      callbacks_count_ = 0;
      auto timer = create_timer(1ns);

      rclcpp::sleep_for(wait_time);

      EXPECT_FALSE(timer.is_running());
      EXPECT_EQ(callbacks_count_, 0);
    }
  }


  INSTANTIATE_TEST_SUITE_P(I, NoAutostart, ::testing::Values(TestTimerType::ros, TestTimerType::thread), &test_timer_type_info_to_string);


  class TimerResetTest : public mrs_lib_testing::RosExecutorFixture<>, public ::testing::WithParamInterface<TestTimerType>
  {
  public:
    TimerResetTest()
    {
      node_ = std::make_shared<rclcpp::Node>("test_timer_handler");
    }

    mrs_lib::Timer create_timer(std::chrono::nanoseconds period)
    {
      switch (GetParam())
      {
      case TestTimerType::ros: {
        auto opts = mrs_lib::RosTimerOptions{
            .node_interfaces = *node_,
            .autostart = true,
            .oneshot = false,
        };
        return mrs_lib::Timer(opts, period, [this]() { callback(); });
      }
      case TestTimerType::thread: {
        auto opts = mrs_lib::ThreadTimerOptions{
            .node_interfaces = *node_,
            .autostart = true,
            .oneshot = false,
        };
        return mrs_lib::Timer(opts, period, [this]() { callback(); });
      }
      }
      throw std::logic_error("Unhandled timer type");
    }

    void reset_test_state()
    {
      callbacks_count_ = 0;
    }

    size_t get_callbacks_count()
    {
      return callbacks_count_;
    }

  private:
    void callback()
    {
      callbacks_count_++;
    }

    std::shared_ptr<rclcpp::Node> node_;

    std::atomic<size_t> callbacks_count_ = 0;
  };

  TEST_P(TimerResetTest, StartOrReset)
  {
    for (int i = 0; i < test_repetitions_count; i++)
    {
      std::cout << "\tStarting test...\n" << std::flush;
      reset_test_state();

      const std::chrono::microseconds wait_time = 50ms;
      const std::chrono::microseconds period = wait_time * 2;

      auto timer = create_timer(period);
      ASSERT_TRUE(timer.is_running());

      // We periodically reset the timer, so it should never trigger.
      for (size_t i = 0; i < 10; ++i)
      {
        std::this_thread::sleep_for(wait_time);
        EXPECT_TRUE(timer.is_running());
        timer.start_or_reset();
        EXPECT_EQ(get_callbacks_count(), 0) << "In iteration: " << i;
        EXPECT_TRUE(timer.is_running());
      }

      rclcpp::sleep_for(wait_time);

      timer.stop();

      EXPECT_FALSE(timer.is_running());
      EXPECT_EQ(get_callbacks_count(), 0);
    }
  }

  TEST_P(TimerResetTest, SetPeriod)
  {
    for (int i = 0; i < test_repetitions_count; i++)
    {
      std::cout << "\tStarting test...\n" << std::flush;
      reset_test_state();

      const std::chrono::microseconds wait_time = 50ms;
      const std::chrono::microseconds period = wait_time * 2;

      auto timer = create_timer(period);
      ASSERT_TRUE(timer.is_running());

      // We periodically reset the timer, so it should never trigger.
      for (size_t i = 0; i < 10; ++i)
      {
        std::this_thread::sleep_for(wait_time);
        EXPECT_TRUE(timer.is_running());
        timer.set_period(period);
        EXPECT_EQ(get_callbacks_count(), 0) << "In iteration: " << i;
        EXPECT_TRUE(timer.is_running());
      }

      rclcpp::sleep_for(wait_time);

      timer.stop();

      EXPECT_FALSE(timer.is_running());
      EXPECT_EQ(get_callbacks_count(), 0);
    }
  }

  INSTANTIATE_TEST_SUITE_P(I, TimerResetTest, ::testing::Values(TestTimerType::ros, TestTimerType::thread), &test_timer_type_info_to_string);


} // namespace
