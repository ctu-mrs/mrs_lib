#include <mrs_lib/publisher_handler.h>
#include <cmath>
#include <chrono>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <thread>

class TestSubscription : public ::testing::Test {

public:
  void callback1(std_msgs::msg::Int64::ConstSharedPtr msg) {

    RCLCPP_INFO(node_->get_logger(), "message received");

    if (msg->data == num_to_send) {
      num_received++;
    }
  }

protected:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase() {
    rclcpp::shutdown();
  }

  void initialize(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) {

    node_ = std::make_shared<rclcpp::Node>("test_publisher_handler", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    finished_future_ = finished_promise_.get_future();

    main_thread_ = std::thread(&TestSubscription::spin, this);
  }

  void spin() {

    RCLCPP_INFO(node_->get_logger(), "starting spinning");

    executor_->spin();

    RCLCPP_INFO(node_->get_logger(), "stopped spinning");
  }

  void despin() {
    executor_->cancel();

    main_thread_.join();
  }

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  std::thread main_thread_;

  int num_received = 0;

  int num_to_send = 1234;

  std::promise<bool> finished_promise_;
  std::future<bool>  finished_future_;
};

TEST_F(TestSubscription, test_publish) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  bool result = 1;

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "/topic1");

  // | ------------------- create a subscriber ------------------ |

  const std::function<void(const std_msgs::msg::Int64::SharedPtr)> callback1_ptr = std::bind(&TestSubscription::callback1, this, std::placeholders::_1);

  auto sub1 = node_->create_subscription<std_msgs::msg::Int64>("/topic1", 100, callback1_ptr);

  // | ---------------------- start testing --------------------- |

  RCLCPP_INFO(node_->get_logger(), "initialized");

  {
    for (int i = 0; i < 10; i++) {

      if (sub1->get_publisher_count() > 0) {
        break;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  if (sub1->get_publisher_count() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "failed to connect publisher and subscriber");
    result &= false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  std_msgs::msg::Int64 data;
  data.data = num_to_send;

  {
    for (int i = 0; i < 10; i++) {

      RCLCPP_INFO(node_->get_logger(), "publishing");

      ph_int.publish(data);

      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  if (num_received != 10) {
    RCLCPP_ERROR(node_->get_logger(), "did not received the correct number of messages, %d != %d", num_received, 10);
    result &= false;
  }

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  EXPECT_TRUE(result);
}
