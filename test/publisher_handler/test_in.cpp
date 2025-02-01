#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_lib/integration_test.h>

#include <chrono>

#include <mrs_lib/publisher_handler.h>

#include <std_msgs/msg/int64.hpp>

using namespace std::chrono_literals;

class Test : public IntegrationTest {

public:
  Test() : IntegrationTest() {
  }

  int num_received = 0;

  int num_to_send = 1234;

  bool testPublish();

  void callback1(const std_msgs::msg::Int64::SharedPtr msg);
};

/* callback1() //{ */

void Test::callback1(const std_msgs::msg::Int64::SharedPtr msg) {

  if (msg->data == num_to_send) {
    num_received++;
  }

  RCLCPP_INFO(node_->get_logger(), "received message");
}

//}

/* testPublish() //{ */

bool Test::testPublish(void) {

  bool result = 1;

  num_received = 0;

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(node_, "topic1");

  // | ------------------- create a subscriber ------------------ |

  const std::function<void(const std_msgs::msg::Int64::SharedPtr)> callback1_ptr = std::bind(&Test::callback1, this, std::placeholders::_1);

  auto sub1 = node_->create_subscription<std_msgs::msg::Int64>("topic1", 10, callback1_ptr);

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
    RCLCPP_ERROR(node_->get_logger(), "did not received the right number of messages, %d != %d", num_received, 10);
    result &= false;
  }

  RCLCPP_INFO(node_->get_logger(), "finished");

  return result;
}

//}

/* bool Test::testThrottle(void) { */

/*   int result = 1; */

/*   int num_received = 0; */

/*   ros::NodeHandle nh = ros::NodeHandle("~"); */

/*   ros::Time::waitForValid(); */

/*   // | ---------------- create publisher handler ---------------- | */

/*   mrs_lib::PublisherHandler<std_msgs::msg::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::msg::Int64>(nh, "topic1", 10, false, 10.0); */

/*   // | ------------------- create a subscriber ------------------ | */

/*   ros::Subscriber sub1 = nh.subscribe<std_msgs::msg::Int64>("topic1", 10, &callback1); */

/*   // | -------------- initialize the async spinner -------------- | */

/*   ros::AsyncSpinner spinner(10); */
/*   spinner.start(); */

/*   // | ---------------------- start testing --------------------- | */

/*   ROS_INFO("[%s]: initialized", ros::this_node::getName().c_str()); */

/*   for (int i = 0; i < 10; i++) { */
/*     if (sub1.getNumPublishers() > 0) { */
/*       break; */
/*     } */
/*     ros::Duration(1.0).sleep(); */
/*   } */

/*   if (sub1.getNumPublishers() == 0) { */
/*     ROS_ERROR("[%s]: failed to connect publisher and subscriber", ros::this_node::getName().c_str()); */
/*     result *= 0; */
/*   } */

/*   ros::Duration(1.0).sleep(); */

/*   std_msgs::msg::Int64 data; */
/*   data.data = num_to_send; */

/*   for (int i = 0; i < 100; i++) { */
/*     ROS_INFO("[%s]: publishing", ros::this_node::getName().c_str()); */
/*     ph_int.publish(data); */
/*     ros::Duration(0.01).sleep(); */
/*   } */

/*   ros::Duration(1.0).sleep(); */

/*   if (!(num_received >= 9 && num_received <= 11)) { */
/*     ROS_ERROR("[%s]: did not received the right number of messages, 9 !=> %d !<= 11", ros::this_node::getName().c_str(), num_received); */
/*     result *= 0; */
/*   } */

/*   ROS_INFO("[%s]: finished", ros::this_node::getName().c_str()); */

/*   EXPECT_TRUE(result); */
/* } */

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Test test;

  test_result &= test.testPublish();

  test.reportTesResult(test_result);

  test.join();
}
