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
};

class Test : public ::testing::TestWithParam<TimerTestData>
{

public:
  /* SetUpTestSuite() //{ */

  static void SetUpTestSuite()
  {
    std::cout << "Ros initialized.\n";
    rclcpp::init(0, nullptr);
  }

  //}

  /* TearDownTestSuite() //{ */

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
    std::cout << "Ros shut down.\n";
  }

  //}

protected:
  /* SetUp() //{ */

  void SetUp() override
  {
    const rclcpp::NodeOptions& node_options = GetParam().node_options;
    rate_ = GetParam().rate;
    callback_sleep_time_ = GetParam().callback_sleep_time;

    node_ = std::make_shared<rclcpp::Node>("test_timer_handler", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    finished_future_ = finished_promise_.get_future();

    main_thread_ = std::thread(&Test::spin, this);
    while (!executor_->is_spinning())
    {
      std::cout << "Waiting for executor to start...\n";
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

  void initializeTimer()
  {
    RCLCPP_INFO(node_->get_logger(), "Constructing timer");
    mrs_lib::TimerHandlerOptions opts;

    opts.node = node_;
    opts.autostart = false;

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
    cbks_ok_ = true;

    timer_stop_called_ = false;

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
  int n_cbks_ = 0;
  bool cbks_in_time_ = true;
  bool null_cbk_ = false;
  bool cbks_ok_ = true;

  std::mutex cbk_running_mtx_;
  bool timer_stop_called_ = false;

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

    if (std::abs(dt.seconds()) > (period / 10.0))
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

  if (timer_stop_called_)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback called while timer is not running!");
    cbks_ok_ = false;
  }

  if (test_stop_from_cbk_)
  {
    timer_->stop();
    std::cout << "\tStopping timer from callback." << std::endl;
    // Mark the timer stopped to test that there are no further callback
    timer_stop_called_ = true;
  }

  rclcpp::sleep_for(callback_sleep_time_);
}

//}

TEST_P(Test, TestCallbackPeriod)
{
  initializeTimer();

  for (int i = 0; i < test_repetitions_count; i++)
  {
    std::cout << "\tTesting callback period\n";
    resetTestState();

    timer_->start();

    const rclcpp::Time start = node_->get_clock()->now();
    const double test_dur = 1.0;

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * test_dur)));

    timer_->stop();
    // There may be a callback running while calling the stop.
    // Wait for it to end and then enable checking for unwanted callbacks
    {
      std::scoped_lock lck(cbk_running_mtx_);
      timer_stop_called_ = true;
    }
    // Wait to see if there are more callbacks after stop
    const size_t wait_periods = 4;
    rclcpp::sleep_for(wait_periods * std::chrono::milliseconds(static_cast<int>(1 / rate_)));

    const double expected_cbks = test_dur * rate_;
    const bool callbacks_in_time = cbks_in_time_;
    const bool callback_while_not_running = !cbks_ok_;

    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE(std::abs(n_cbks_ - expected_cbks), 2);
    EXPECT_TRUE(callbacks_in_time);

    n_cbks_ = 0;

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * (1.0 / rate_) * 2)));

    const bool no_callbacks_after_stopped = n_cbks_ == 0;
    EXPECT_TRUE(no_callbacks_after_stopped);
  }

  destroyTimer();
}


TEST_P(Test, StopFromCallback)
{
  initializeTimer();
  test_stop_from_cbk_ = true;

  for (int i = 0; i < test_repetitions_count; i++)
  {
    std::cout << "\tTesting stop from callback\n";
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
    std::cout << "\tTesting destructor\n";

    resetTestState();

    const rclcpp::Time start = node_->get_clock()->now();

    timer_->setPeriod(rclcpp::Duration(std::chrono::duration<double>(0.025)));
    timer_->start();
    destroyTimer();

    const rclcpp::Time destroyed = node_->get_clock()->now();
    const bool callback_while_destroyed = null_cbk_;
    const bool callback_while_not_running = !cbks_ok_;

    EXPECT_FALSE(callback_while_destroyed);
    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE((destroyed - start).seconds(), 1.0);
  }
}

INSTANTIATE_TEST_SUITE_P(ThreadTimerInstance, Test,
                         ::testing::Values(TimerTestData{
                             .timer_type = TestTimerType::thread,
                             .node_options = rclcpp::NodeOptions().use_intra_process_comms(false),
                             .rate = 50,
                             .callback_sleep_time = std::chrono::milliseconds(10),
                         }));


INSTANTIATE_TEST_SUITE_P(RosTimerInstance, Test,
                         ::testing::Values(TimerTestData{
                             .timer_type = TestTimerType::ros,
                             .node_options = rclcpp::NodeOptions().use_intra_process_comms(false),
                             .rate = 50,
                             .callback_sleep_time = std::chrono::milliseconds(10),
                         }));
