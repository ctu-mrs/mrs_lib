#include <mrs_lib/publisher_handler.h>
#include <cmath>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/timer_handler.h>

#include <thread>

using namespace std::chrono_literals;

class TimerHandler : public ::testing::Test
{

public:
protected:
  /* SetUpTestCase() //{ */

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  //}

  /* TearDownTestCase() //{ */

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  //}

  /* initialize() //{ */

  void initialize(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
  {

    node_ = std::make_shared<rclcpp::Node>("test_publisher_handler", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    finished_future_ = finished_promise_.get_future();

    main_thread_ = std::thread(&TimerHandler::spin, this);
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

  /* despin() //{ */

  void despin()
  {
    executor_->cancel();

    main_thread_.join();
  }

  //}

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  std::promise<bool> finished_promise_;
  std::future<bool> finished_future_;

  void do_test(const bool use_threadtimer);
};

struct obj_t
{
  obj_t(rclcpp::Node::SharedPtr node, std::shared_ptr<mrs_lib::MRSTimer> timer, std::shared_ptr<std::atomic<bool>> test_stop_from_cbk)
      : node_(node), timer_(timer), test_stop_from_cbk_(test_stop_from_cbk)
  {

    last_time_callback_ = node_->get_clock()->now();
  }

  double r = 50.0;
  int n_cbks = 0;
  bool cbks_in_time = true;
  bool null_cbk = false;
  bool cbks_ok = true;
  std::mutex cbk_running_mtx;
  std::atomic<bool> cbk_running = false;

  rclcpp::Time last_time_callback_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<mrs_lib::MRSTimer> timer_;

  std::shared_ptr<std::atomic<bool>> test_stop_from_cbk_;

  void setTimer(std::shared_ptr<mrs_lib::MRSTimer> timer)
  {

    this->timer_ = timer;
    std::cout << "timer set" << std::endl;
  }

  void callback()
  {

    std::cout << "callback called" << std::endl;

    std::scoped_lock lck(cbk_running_mtx);

    mrs_lib::AtomicScopeFlag running(cbk_running);
    n_cbks++;

    double expected_time = (last_time_callback_ + rclcpp::Duration(std::chrono::duration<double>(1 / r))).seconds();
    double now = node_->get_clock()->now().seconds();

    double dt = now - last_time_callback_.seconds();

    // this is quite a large tolerance, but the ROS timer actually isn't capable of performing better, so it has to be
    if (std::abs(dt) > (r / 2.0))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback did not come in time! Expected: " << expected_time << ", received: " << now << " (period: " << 1.0 / r
                                                                                           << ", difference: " << dt << ")");
      cbks_in_time = false;
    }

    if (!timer_)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback called while timer is already destroyed!");
      null_cbk = true;
      return;
    }

    if (!timer_->running())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Callback called while timer is not running!");
      cbks_ok = false;
    }

    if (*test_stop_from_cbk_)
    {
      timer_->stop();
      std::cout << "\tStopping timer from callback." << std::endl;
    }
  }
};

/* TEST_F(TimerHandler, thread_timer) //{ */

void TimerHandler::do_test(const bool use_threadtimer)
{

  std::shared_ptr<std::atomic<bool>> test_stop_from_cbk = std::make_shared<std::atomic<bool>>(false);

  std::cout << "\tConstructing timer\n";

  std::shared_ptr<mrs_lib::MRSTimer> timer;

  mrs_lib::TimerHandlerOptions opts;

  opts.node = node_;
  opts.autostart = false;

  double rate = 50.0;  // Hz

  obj_t cbk_obj(node_, timer, test_stop_from_cbk);

  std::function<void()> callback_fcn = std::bind(&obj_t::callback, &cbk_obj);

  if (use_threadtimer)
  {
    timer = std::make_shared<mrs_lib::ThreadTimer>(opts, rate, callback_fcn);
  } else
  {
    timer = std::make_shared<mrs_lib::ROSTimer>(opts, rate, callback_fcn);
  }

  cbk_obj.setTimer(timer);

  {
    std::cout << "\tTesting callback period\n";
    timer->start();

    const rclcpp::Time start = node_->get_clock()->now();
    const double test_dur = 1.0;

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * test_dur)));

    timer->stop();
    EXPECT_FALSE(cbk_obj.cbk_running);

    const double expected_cbks = test_dur * rate;
    const bool callbacks_in_time = cbk_obj.cbks_in_time;
    const bool callback_while_not_running = !cbk_obj.cbks_ok;

    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE(std::abs(cbk_obj.n_cbks - expected_cbks), 2);
    EXPECT_TRUE(callbacks_in_time);

    cbk_obj.n_cbks = 0;

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * (1.0 / rate) * 2)));

    const bool no_callbacks_after_stopped = cbk_obj.n_cbks == 0;
    EXPECT_TRUE(no_callbacks_after_stopped);
  }

  {
    std::cout << "\tTesting stop from callback\n";
    cbk_obj.n_cbks = 0;
    *test_stop_from_cbk = true;
    timer->start();

    // wait for one callback to be called
    while (!cbk_obj.n_cbks)
    {
      rclcpp::sleep_for(1s);
    }

    timer->stop();

    // wait for the callback to end
    {
      std::scoped_lock lck(cbk_obj.cbk_running_mtx);
      cbk_obj.n_cbks = 0;
      const bool timer_stopped_from_callback = !timer->running();
      EXPECT_TRUE(timer_stopped_from_callback);
    }

    rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * (1.0 / rate) * 2)));

    const bool no_callbacks_after_stopped = cbk_obj.n_cbks == 0;
    EXPECT_TRUE(no_callbacks_after_stopped);
  }

  {
    std::cout << "\tTesting destructor\n";

    cbk_obj.cbks_ok = true;

    const rclcpp::Time start = node_->get_clock()->now();

    const double period = 1.0 / 50;

    timer->setPeriod(period);
    timer->start();
    timer = nullptr;

    const rclcpp::Time destroyed = node_->get_clock()->now();
    const bool callback_while_destroyed = cbk_obj.null_cbk;
    const bool callback_while_not_running = !cbk_obj.cbks_ok;

    EXPECT_FALSE(callback_while_destroyed);
    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE((destroyed - start).seconds(), 1.0);
  }
}

TEST_F(TimerHandler, thread_timer)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  bool result = 1;

  RCLCPP_INFO(node_->get_logger(), "finished");

  bool use_threadtimer = true;

  for (int i = 0; i < 8; i++) {
    do_test(use_threadtimer);
  }

  despin();

  clock->sleep_for(1s);

  EXPECT_TRUE(result);
}

TEST_F(TimerHandler, ros_timer)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  bool result = 1;

  RCLCPP_INFO(node_->get_logger(), "finished");

  bool use_threadtimer = false;

  for (int i = 0; i < 8; i++) {
    do_test(use_threadtimer);
  }

  despin();

  clock->sleep_for(1s);

  EXPECT_TRUE(result);
}

//}
