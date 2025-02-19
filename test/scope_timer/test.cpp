#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/scope_timer.h>

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

    node_ = std::make_shared<rclcpp::Node>("test_publisher_handler", node_options);

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

  int num_received = 0;

  int num_to_send = 1234;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;
};

/* TEST_F(TimeoutManager, test_timeout_manager) //{ */

TEST_F(Test, test_basic) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  {
    std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_ = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, "", false);

    mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(node_, "test_basic", scope_timer_logger_, true);

    clock->sleep_for(1s);

    timer.checkpoint("check1");

    clock->sleep_for(1.1s);

    double lifetime = timer.getLifetime();

    ASSERT_GT(lifetime, 2000);
  }

  std::cout << "Done" << std::endl;

  despin();

  clock->sleep_for(1s);
}

//}
