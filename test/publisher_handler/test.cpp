#include <cmath>
#include <chrono>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/publisher_handler.h>

#include <thread>

using namespace std::chrono_literals;

class Test : public ::testing::Test {

public:
  /* callback1() //{ */

  void callback1(std_msgs::msg::Int64::ConstSharedPtr msg) {

    RCLCPP_INFO(node_->get_logger(), "message received");

    if (msg->data == num_to_send) {
      num_received++;
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

  int num_received = 0;

  int num_to_send = 1234;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;
};

/* TEST_F(Test, test_publish) //{ */

TEST_F(Test, test_publish) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  bool result = 1;

  RCLCPP_INFO(node_->get_logger(), "creating publisher");

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "creating subscriber");

  const std::function<void(const std_msgs::msg::Int64::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

  auto sub1 = node_->create_subscription<std_msgs::msg::Int64>("/topic1", 100, callback1_ptr);

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    rclcpp::Rate rate(1.0, clock);

    for (int i = 0; i < 10; i++) {

      if (sub1->get_publisher_count() > 0) {
        break;
      }

      rate.sleep();
    }
  }

  if (sub1->get_publisher_count() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    result &= false;
  }

  clock->sleep_for(1s);

  std_msgs::msg::Int64 data;
  data.data = num_to_send;

  {

    rclcpp::Rate rate(100.0, clock);

    for (int i = 0; i < 10; i++) {

      RCLCPP_INFO(node_->get_logger(), "publishing");

      ph_int.publish(data);

      rate.sleep();
    }
  }

  clock->sleep_for(1s);

  if (num_received != 10) {
    RCLCPP_ERROR(node_->get_logger(), "did not received the correct number of messages, %d != %d", num_received, 10);
    result &= false;
  }

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();

  clock->sleep_for(1s);

  EXPECT_TRUE(result);
}

//}

/* TEST_F(Test, test_opts) //{ */

TEST_F(Test, test_opts) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  bool result = 1;

  RCLCPP_INFO(node_->get_logger(), "creating publisher");

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandlerOptions opts;

  opts.node = node_;

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(opts, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "creating subscriber");

  const std::function<void(const std_msgs::msg::Int64::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

  auto sub1 = node_->create_subscription<std_msgs::msg::Int64>("/topic1", 100, callback1_ptr);

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    for (int i = 0; i < 10; i++) {

      if (sub1->get_publisher_count() > 0) {
        break;
      }

      clock->sleep_for(1s);
    }
  }

  if (sub1->get_publisher_count() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    result &= false;
  }

  rclcpp::sleep_for(1s);

  std_msgs::msg::Int64 data;
  data.data = num_to_send;

  {
    rclcpp::Rate rate(10, clock);

    for (int i = 0; i < 10; i++) {

      RCLCPP_INFO(node_->get_logger(), "publishing");

      ph_int.publish(data);

      rate.sleep();
    }
  }

  rclcpp::sleep_for(1s);

  if (num_received != 10) {
    RCLCPP_ERROR(node_->get_logger(), "did not received the correct number of messages, %d != %d", num_received, 10);
    result &= false;
  }

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();

  rclcpp::sleep_for(1s);

  EXPECT_TRUE(result);
}

//}

/* TEST_F(Test, throttling) //{ */

TEST_F(Test, throttling) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  bool result = 1;

  RCLCPP_INFO(node_->get_logger(), "creating publisher");

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandlerOptions opts;

  opts.node          = node_;
  opts.throttle_rate = 10.0;

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(opts, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  RCLCPP_INFO(node_->get_logger(), "creating subscriber");

  const std::function<void(const std_msgs::msg::Int64::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

  auto sub1 = node_->create_subscription<std_msgs::msg::Int64>("/topic1", 100, callback1_ptr);

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    for (int i = 0; i < 10; i++) {

      if (sub1->get_publisher_count() > 0) {
        break;
      }

      rclcpp::sleep_for(1s);
    }
  }

  if (sub1->get_publisher_count() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    result &= false;
  }

  rclcpp::sleep_for(1s);

  std_msgs::msg::Int64 data;
  data.data = num_to_send;

  {
    rclcpp::Rate rate(100, clock);

    for (int i = 0; i < 100; i++) {

      RCLCPP_INFO(node_->get_logger(), "publishing");

      ph_int.publish(data);

      rate.sleep();
    }
  }

  rclcpp::sleep_for(1s);

  if (num_received < 9 || num_received > 11) {
    RCLCPP_ERROR(node_->get_logger(), "did not received the correct number of messages, want 9 >= %d <= 11", num_received);
    result &= false;
  }

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();

  rclcpp::sleep_for(1s);

  EXPECT_TRUE(result);
}

//}
