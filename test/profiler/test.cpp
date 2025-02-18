#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/profiler.h>

#include <thread>

using namespace std::chrono_literals;

class Test : public ::testing::Test {

public:
  /* callback1() //{ */

  void callback1(mrs_msgs::msg::ProfilerUpdate::ConstSharedPtr msg) {

    RCLCPP_INFO(node_->get_logger(), "message received");

    received_ = true;

    if (msg->duration > 0.5 && msg->duration < 1.5) {
      received_ = false;
    }

    if (msg->node_name != "Test") {
      received_ = false;
    }

    if (msg->routine_name != "test_routine") {
      received_ = false;
    }
  }

  //}

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

  std::atomic<bool> received_ = false;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;

  mrs_lib::Profiler profiler_;
};

/* TEST_F(Test, test_basic) //{ */

TEST_F(Test, test_basic) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  profiler_ = mrs_lib::Profiler(node_, "Test", true);

  clock->sleep_for(0.1s);

  // | ------------------- create a subscriber ------------------ |

  const std::function<void(const mrs_msgs::msg::ProfilerUpdate::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

  auto sub1 = node_->create_subscription<mrs_msgs::msg::ProfilerUpdate>("profiler", 100, callback1_ptr);

  // | -------------------------- wait -------------------------- |

  clock->sleep_for(0.1s);

  // | -------------------------- test -------------------------- |

  {
    profiler_.createRoutine("test_routine");

    clock->sleep_for(1s);
  }

  clock->sleep_for(0.1s);

  EXPECT_TRUE(received_);

  std::cout << "Done" << std::endl;

  despin();

  clock->sleep_for(1s);
}

//}
//
/* TEST_F(Test, test_not_active) //{ */

TEST_F(Test, test_not_active) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  profiler_ = mrs_lib::Profiler(node_, "Test", false);

  clock->sleep_for(0.1s);

  // | ------------------- create a subscriber ------------------ |

  const std::function<void(const mrs_msgs::msg::ProfilerUpdate::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

  auto sub1 = node_->create_subscription<mrs_msgs::msg::ProfilerUpdate>("profiler", 100, callback1_ptr);

  // | -------------------------- wait -------------------------- |

  clock->sleep_for(0.1s);

  // | -------------------------- test -------------------------- |

  {
    profiler_.createRoutine("test_routine");

    clock->sleep_for(1s);
  }

  clock->sleep_for(0.1s);

  EXPECT_FALSE(received_);

  std::cout << "Done" << std::endl;

  despin();

  clock->sleep_for(1s);
}

//}
