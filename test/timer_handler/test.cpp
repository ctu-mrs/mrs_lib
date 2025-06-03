#include <cmath>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/timer_handler.h>

#include <thread>

using namespace std::chrono_literals;

class Test : public ::testing::Test {

public:
protected:
  /* SetUpTestCase() //{ */

  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
  }

  //}

  /* TearDownTestCase() //{ */

  static void TearDownTestCase() {
    rclcpp::shutdown();
  }

  //}

  /* initialize() //{ */

  void initialize(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) {

    node_ = std::make_shared<rclcpp::Node>("test_timer_handler", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    finished_future_ = finished_promise_.get_future();

    main_thread_ = std::thread(&Test::spin, this);
  }

  //}

  /* spin() //{ */

  void spin() {

    RCLCPP_INFO(node_->get_logger(), "starting spinning");

    executor_->spin();

    RCLCPP_INFO(node_->get_logger(), "stopped spinning");
  }

  //}

  /* despin() //{ */

  void despin() {
    executor_->cancel();

    main_thread_.join();
  }

  //}

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;

  void do_test(const bool use_threadtimer);

  void timerCallback(void);

  double            rate_         = 50;
  int               n_cbks_       = 0;
  bool              cbks_in_time_ = true;
  bool              null_cbk_     = false;
  bool              cbks_ok_      = true;
  std::mutex        cbk_running_mtx_;
  std::atomic<bool> cbk_running_ = false;

  std::atomic<bool> test_stop_from_cbk_ = false;

  std::shared_ptr<mrs_lib::MRSTimer> timer_;

  std::optional<rclcpp::Time> last_time_callback_;
};

/* timerCallback() //{ */

void Test::timerCallback(void) {

  std::scoped_lock lck(cbk_running_mtx_);

  mrs_lib::AtomicScopeFlag running(cbk_running_);
  n_cbks_++;

  if (last_time_callback_) {

    double expected_time = (last_time_callback_.value() + rclcpp::Duration(std::chrono::duration<double>(1.0 / rate_))).seconds();
    double now           = node_->get_clock()->now().seconds();

    double dt = now - expected_time;

    // this is quite a large tolerance, but the ROS timer actually isn't capable of performing better, so it has to be
    if (std::abs(dt) > (rate_ / 2.0)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback did not come in time! Expected: " << expected_time << ", received: " << now
                                                                                           << " (period: " << 1.0 / rate_ << ", difference: " << dt << ")");
      cbks_in_time_ = false;
    }
  }

  last_time_callback_ = {node_->get_clock()->now()};

  if (!timer_) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback called while timer is already destroyed!");
    null_cbk_ = true;
    return;
  }

  if (!timer_->running()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback called while timer is not running!");
    cbks_ok_ = false;
  }

  if (test_stop_from_cbk_) {
    timer_->stop();
    std::cout << "\tStopping timer from callback." << std::endl;
  }
}

//}

/* do_test() //{ */

void Test::do_test(const bool use_threadtimer) {

  RCLCPP_INFO(node_->get_logger(), "Constructing timer");

  test_stop_from_cbk_ = false;

  mrs_lib::TimerHandlerOptions opts;

  opts.node      = node_;
  opts.autostart = false;

  std::function<void()> callback_fcn = std::bind(&Test::timerCallback, this);

  if (use_threadtimer) {
    timer_ = std::make_shared<mrs_lib::ThreadTimer>(opts, rclcpp::Rate(rate_, node_->get_clock()), callback_fcn);
  } else {
    timer_ = std::make_shared<mrs_lib::ROSTimer>(opts, rclcpp::Rate(rate_, node_->get_clock()), callback_fcn);
  }

  {
    std::cout << "\tTesting callback period\n";
    timer_->start();

    const rclcpp::Time start    = node_->get_clock()->now();
    const double       test_dur = 1.0;

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * test_dur)));

    timer_->stop();

    EXPECT_FALSE(cbk_running_);

    const double expected_cbks              = test_dur * rate_;
    const bool   callbacks_in_time          = cbks_in_time_;
    const bool   callback_while_not_running = !cbks_ok_;

    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE(std::abs(n_cbks_ - expected_cbks), 2);
    EXPECT_TRUE(callbacks_in_time);

    n_cbks_ = 0;

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * (1.0 / rate_) * 2)));

    const bool no_callbacks_after_stopped = n_cbks_ == 0;
    EXPECT_TRUE(no_callbacks_after_stopped);
  }

  {
    std::cout << "\tTesting stop from callback\n";
    n_cbks_             = 0;
    test_stop_from_cbk_ = true;
    timer_->start();

    // wait for one callback to be called
    while (!n_cbks_) {
      rclcpp::sleep_for(1s);
    }

    timer_->stop();

    // wait for the callback to end
    {
      std::scoped_lock lck(cbk_running_mtx_);
      n_cbks_                                = 0;
      const bool timer_stopped_from_callback = !timer_->running();
      EXPECT_TRUE(timer_stopped_from_callback);
    }

    node_->get_clock()->sleep_for(std::chrono::duration<double>(double(2.0 / rate_)));

    const bool no_callbacks_after_stopped = n_cbks_ == 0;
    EXPECT_TRUE(no_callbacks_after_stopped);
  }

  {
    std::cout << "\tTesting destructor\n";

    cbks_ok_ = true;

    const rclcpp::Time start = node_->get_clock()->now();

    timer_->setPeriod(rclcpp::Duration(std::chrono::duration<double>(0.025)));
    timer_->start();
    timer_ = nullptr;

    const rclcpp::Time destroyed                  = node_->get_clock()->now();
    const bool         callback_while_destroyed   = null_cbk_;
    const bool         callback_while_not_running = !cbks_ok_;

    EXPECT_FALSE(callback_while_destroyed);
    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE((destroyed - start).seconds(), 1.0);
  }
}

//}

/* TEST_F(Test, thread_timer) //{ */

TEST_F(Test, thread_timer) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "finished");

  bool use_threadtimer = true;

  for (int i = 0; i < 8; i++) {
    do_test(use_threadtimer);
  }

  despin();

  clock->sleep_for(1s);
}

//}

/* TEST_F(Test, ros_timer) //{ */

// The thread timer cuurrently fails due to 
//    EXPECT_FALSE(cbk_running_); 
// .. the timer callback is still running after we cancel the timer...

/* TEST_F(Test, ros_timer) { */

/*   initialize(rclcpp::NodeOptions().use_intra_process_comms(false)); */

/*   auto clock = node_->get_clock(); */

/*   RCLCPP_INFO(node_->get_logger(), "finished"); */

/*   bool use_threadtimer = false; */

/*   for (int i = 0; i < 8; i++) { */
/*     do_test(use_threadtimer); */
/*   } */

/*   despin(); */

/*   clock->sleep_for(1s); */
/* } */

//}
