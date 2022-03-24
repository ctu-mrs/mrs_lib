#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_lib/publisher_handler.h>

#include <string>
#include <thread>

#include <std_msgs/Int64.h>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

int num_received = 0;

int num_to_send = 1234;

/* callback1() //{ */

void callback1(const std_msgs::Int64::ConstPtr& msg) {

  if (msg->data == num_to_send) {
    num_received++;
  }

  ROS_INFO("[%s]: received message", ros::this_node::getName().c_str());
}

//}

/* TEST(TESTSuite, publish_test) //{ */

TEST(TESTSuite, publish_test) {

  int result = 1;

  num_received = 0;

  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::Int64>(nh, "topic1");

  // | ------------------- create a subscriber ------------------ |

  ros::Subscriber sub1 = nh.subscribe<std_msgs::Int64>("topic1", 10, &callback1);

  // | -------------- initialize the async spinner -------------- |

  ros::AsyncSpinner spinner(10);
  spinner.start();

  // | ---------------------- start testing --------------------- |

  ROS_INFO("[%s]: initialized", ros::this_node::getName().c_str());

  for (int i = 0; i < 10; i++) {
    if (sub1.getNumPublishers() > 0) {
      break;
    }
    ros::Duration(1.0).sleep();
  }

  if (sub1.getNumPublishers() == 0) {
    ROS_ERROR("[%s]: failed to connect publisher and subscriber", ros::this_node::getName().c_str());
    result *= 0;
  }

  ros::Duration(1.0).sleep();

  std_msgs::Int64 data;
  data.data = num_to_send;

  for (int i = 0; i < 10; i++) {
    ROS_INFO("[%s]: publishing", ros::this_node::getName().c_str());
    ph_int.publish(data);
    ros::Duration(0.01).sleep();
  }

  ros::Duration(1.0).sleep();

  if (num_received != 10) {
    ROS_ERROR("[%s]: did not received the right number of messages, %d != %d", ros::this_node::getName().c_str(), num_received, 10);
    result *= 0;
  }

  ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, throttle_test) //{ */

TEST(TESTSuite, throttle_test) {

  int result = 1;

  num_received = 0;

  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // | ---------------- create publisher handler ---------------- |

  mrs_lib::PublisherHandler<std_msgs::Int64> ph_int = mrs_lib::PublisherHandler<std_msgs::Int64>(nh, "topic1", 10, false, 10.0);

  // | ------------------- create a subscriber ------------------ |

  ros::Subscriber sub1 = nh.subscribe<std_msgs::Int64>("topic1", 10, &callback1);

  // | -------------- initialize the async spinner -------------- |

  ros::AsyncSpinner spinner(10);
  spinner.start();

  // | ---------------------- start testing --------------------- |

  ROS_INFO("[%s]: initialized", ros::this_node::getName().c_str());

  for (int i = 0; i < 10; i++) {
    if (sub1.getNumPublishers() > 0) {
      break;
    }
    ros::Duration(1.0).sleep();
  }

  if (sub1.getNumPublishers() == 0) {
    ROS_ERROR("[%s]: failed to connect publisher and subscriber", ros::this_node::getName().c_str());
    result *= 0;
  }

  ros::Duration(1.0).sleep();

  std_msgs::Int64 data;
  data.data = num_to_send;

  for (int i = 0; i < 100; i++) {
    ROS_INFO("[%s]: publishing", ros::this_node::getName().c_str());
    ph_int.publish(data);
    ros::Duration(0.01).sleep();
  }

  ros::Duration(1.0).sleep();

  if (!(num_received >= 9 && num_received <= 11)) {
    ROS_ERROR("[%s]: did not received the right number of messages, 9 !=> %d !<= 11", ros::this_node::getName().c_str(), num_received);
    result *= 0;
  }

  ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "PublisherHandlerTest");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
