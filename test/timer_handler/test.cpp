#include <cmath>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/timer_handler.h>

#include <thread>

using namespace std::chrono_literals;

namespace
{
  constexpr int test_repetitions_count = 8;
}

enum class TestTimerType
{
  ros,
  thread
};

struct TimerTestData
{
  TestTimerType timer_type;
  rclcpp::NodeOptions node_options;
  double rate;
  std::chrono::nanoseconds callback_sleep_time;
  std::chrono::nanoseconds max_drift;
};

class Test : public ::testing::TestWithParam<TimerTestData>
{

public:
  /* SetUpTestSuite() //{ */

  static void SetUpTestSuite()
  {
    std::cout << "Ros initialized.\n" << std::flush;
    rclcpp::init(0, nullptr);
  }

  //}

  /* TearDownTestSuite() //{ */

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
    std::cout << "Ros shut down.\n" << std::flush;
  }

  //}

protected:
  /* SetUp() //{ */

  void SetUp() override
  {
    const rclcpp::NodeOptions& node_options = GetParam().node_options;
    rate_ = GetParam().rate;
    callback_sleep_time_ = GetParam().callback_sleep_time;
    max_drift_ = GetParam().max_drift;

    node_ = std::make_shared<rclcpp::Node>("test_timer_handler", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    finished_future_ = finished_promise_.get_future();

    main_thread_ = std::thread(&Test::spin, this);
    while (!executor_->is_spinning())
    {
      std::cout << "Waiting for executor to start...\n" << std::flush;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  //}

  /* spin() //{ */

  void spin()
  {

    RCLCPP_INFO(node_->get_logger(), "starting spinning");

    executor_->spin();

    RCLCPP_INFO(node_->get_logger(), "stopped spinning");
  }

  //}

  /* TearDown() //{ */

  void TearDown() override
  {
    executor_->cancel();

    main_thread_.join();
    RCLCPP_INFO(node_->get_logger(), "thread joined");
  }

  //}

  mrs_lib::TimerHandlerOptions default_options()
  {
    mrs_lib::TimerHandlerOptions opts;
    opts.node = node_;
    opts.autostart = false;
    return opts;
  }

  void initializeTimer(std::optional<mrs_lib::TimerHandlerOptions> opts_arg = {})
  {
    RCLCPP_INFO(node_->get_logger(), "Constructing timer");

    mrs_lib::TimerHandlerOptions opts = opts_arg.value_or(default_options());

    std::function<void()> callback_fcn = [this]() { this->timerCallback(); };

    switch (GetParam().timer_type)
    {
    case TestTimerType::ros:
      timer_ = std::make_shared<mrs_lib::ROSTimer>(opts, rclcpp::Rate(rate_, node_->get_clock()), callback_fcn);
      break;
    case TestTimerType::thread:
      timer_ = std::make_shared<mrs_lib::ThreadTimer>(opts, rclcpp::Rate(rate_, node_->get_clock()), callback_fcn);
      break;
    default:
      throw std::logic_error("Unhandled timer type");
    }
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

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  std::promise<bool> finished_promise_;
  std::future<bool> finished_future_;

  void timerCallback();

  double rate_;
  std::chrono::nanoseconds callback_sleep_time_;
  std::chrono::nanoseconds max_drift_;

  std::atomic<int> n_cbks_ = 0;
  bool cbks_in_time_ = true;
  bool null_cbk_ = false;
  std::atomic<bool> callback_called_after_stopped_ = false;

  std::mutex cbk_running_mtx_;
  bool timer_stop_called_from_callback_ = false;

  std::atomic<bool> test_stop_from_cbk_ = false;

  std::shared_ptr<mrs_lib::MRSTimer> timer_;

  std::optional<rclcpp::Time> last_time_callback_;
};

/* timerCallback() //{ */

void Test::timerCallback()
{

  std::scoped_lock lck(cbk_running_mtx_);

  n_cbks_++;

  const auto now = node_->get_clock()->now();

  if (last_time_callback_)
  {
    double period = 1.0 / rate_;

    rclcpp::Time expected_time = last_time_callback_.value() + rclcpp::Duration(std::chrono::duration<double>(period));
    rclcpp::Duration dt = now - expected_time;
    last_time_callback_.value() = expected_time;

    double max_dt = std::chrono::duration<double>(max_drift_).count();

    if (std::abs(dt.seconds()) > max_dt)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback did not come in time! Expected: " << expected_time.seconds() << ", received: " << now.seconds()
                                                                                           << " (period: " << period << ", difference: " << dt.seconds()
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

//}

TEST_P(Test, TestCallbackPeriod)
{
  initializeTimer();

  for (int i = 0; i < test_repetitions_count; i++)
  {
    std::cout << "\tTesting callback period\n" << std::flush;
    resetTestState();

    timer_->start();

    const rclcpp::Time start = node_->get_clock()->now();
    const double test_dur = 1.0;

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * test_dur)));

    timer_->stop();
    // There may be a callback running while calling the stop.
    const int callbacks_at_stop = n_cbks_.exchange(0);
    // Wait to see if there are more callbacks after stop
    const size_t wait_periods = 4;
    rclcpp::sleep_for(wait_periods * std::chrono::milliseconds(static_cast<int>(1000 / rate_)));

    const double expected_cbks = test_dur * rate_;
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
  initializeTimer();
  test_stop_from_cbk_ = true;

  for (int i = 0; i < test_repetitions_count; i++)
  {
    std::cout << "\tTesting stop from callback\n" << std::flush;
    resetTestState();
    timer_->start();

    // wait for one callback to be called
    while (!n_cbks_)
    {
      rclcpp::sleep_for(1s);
    }

    // wait for the callback to end
    {
      std::scoped_lock lck(cbk_running_mtx_);
      n_cbks_ = 0;
      const bool timer_stopped_from_callback = !timer_->running();
      EXPECT_TRUE(timer_stopped_from_callback);
    }

    node_->get_clock()->sleep_for(std::chrono::duration<double>(double(2.0 / rate_)));

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
    initializeTimer();
    std::cout << "\tTesting destructor\n" << std::flush;

    resetTestState();

    const rclcpp::Time start = node_->get_clock()->now();

    timer_->setPeriod(rclcpp::Duration(std::chrono::duration<double>(0.025)));
    timer_->start();
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
  if (GetParam().timer_type == TestTimerType::ros)
  {
    GTEST_SKIP() << "Ros timer does not currently handle oneshot!";
  }

  for (int i = 0; i < test_repetitions_count; i++)
  {
    std::cout << "\tTesting oneshot timer\n" << std::flush;

    const size_t wait_periods = 4;
    const auto period = rclcpp::Rate(rate_).period();

    mrs_lib::TimerHandlerOptions opts;
    opts.node = node_;
    opts.autostart = true;
    opts.oneshot = true;

    resetTestState();
    initializeTimer(opts);

    rclcpp::sleep_for(period * wait_periods);

    EXPECT_FALSE(timer_->running());
    EXPECT_EQ(n_cbks_, 1);
    resetTestState();

    rclcpp::sleep_for(period * wait_periods);

    EXPECT_FALSE(timer_->running());
    EXPECT_EQ(n_cbks_, 0);
    resetTestState();

    timer_->start();

    rclcpp::sleep_for(period * wait_periods);

    EXPECT_FALSE(timer_->running());
    EXPECT_EQ(n_cbks_, 1);
    resetTestState();

    destroyTimer();
  }
}

INSTANTIATE_TEST_SUITE_P(ThreadTimerInstance, Test,
                         ::testing::Values(TimerTestData{
                             .timer_type = TestTimerType::thread,
                             .node_options = rclcpp::NodeOptions().use_intra_process_comms(false),
                             .rate = 50,
                             .callback_sleep_time = std::chrono::milliseconds(10),
                             .max_drift = std::chrono::milliseconds(10),
                         }));


INSTANTIATE_TEST_SUITE_P(RosTimerInstance, Test,
                         ::testing::Values(TimerTestData{
                             .timer_type = TestTimerType::ros,
                             .node_options = rclcpp::NodeOptions().use_intra_process_comms(false),
                             .rate = 50,
                             .callback_sleep_time = std::chrono::milliseconds(10),
                             .max_drift = std::chrono::milliseconds(10),
                         }));

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
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void TearDown() override
  {
    executor_->cancel();

    spinning_thread_.join();
    RCLCPP_INFO(node_->get_logger(), "thread joined");
  }

  std::shared_ptr<mrs_lib::MRSTimer> create_timer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate)
  {
    switch (GetParam())
    {
    case TestTimerType::ros:
      return std::make_shared<mrs_lib::ROSTimer>(opts, rate, [this]() { callback(); });
    case TestTimerType::thread:
      return std::make_shared<mrs_lib::ThreadTimer>(opts, rate, [this]() { callback(); });
    default:
      throw std::logic_error("Unhandled timer type");
    }
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
  mrs_lib::TimerHandlerOptions opts;
  opts.node = node_;
  opts.oneshot = false;
  opts.autostart = false;

  for (int i = 0; i < test_repetitions_count; i++)
  {
    std::cout << "\tTesting disabled autostart\n" << std::flush;

    const std::chrono::microseconds wait_time = 100ms;

    callbacks_count_ = 0;
    auto timer = create_timer(opts, rclcpp::Rate(10'000'000'000));

    rclcpp::sleep_for(wait_time);

    EXPECT_FALSE(timer->running());
    EXPECT_EQ(callbacks_count_, 0);
  }
}


INSTANTIATE_TEST_SUITE_P(I, NoAutostart, ::testing::Values(TestTimerType::ros, TestTimerType::thread),
                         [](const ::testing::TestParamInfo<TestTimerType>& info) -> std::string {
                           TestTimerType val = info.param;
                           switch (val)
                           {
                           case TestTimerType::ros:
                             return "ros";
                           case TestTimerType::thread:
                             return "thread";
                           default:
                             throw std::format("unknown_at_idx_{}", info.index);
                           }
                         });
