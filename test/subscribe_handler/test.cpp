#include <geometry_msgs/PointStamped.h>
#include <mrs_lib/subscribe_handler.h>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace mrs_lib;
using namespace std;

bool got_messages_ = false;

void timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs) {
  ROS_ERROR_STREAM("Have not received message from topic '" << topic << "' for " << (ros::Time::now() - last_msg).toSec() << " seconds (" << n_pubs
                                                            << " publishers on topic)");
}

bool started = true;
void message_callback(mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>& sh) {

  if (!started) {
    ROS_ERROR("NOT STARTED, SHOULDN'T HAVE RECEIVED!");
    return;
  }

  ROS_INFO_STREAM("RECEIVED '" << sh.getMsg() << "'");
  got_messages_ = true;
}

/* TEST(TESTSuite, main_test) //{ */

TEST(TESTSuite, main_test) {

  ros::NodeHandle nh("~");
  mrs_lib::SubscribeHandler<geometry_msgs::PointStamped> sh;

  int result = 0;

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.autostart = false;
  /* after this duration without receiving messages on the handled topic, the timeout_callback will be called */
  shopts.no_message_timeout = ros::Duration(2.0);
  /* whether mutexes should be used to prevent data races (set to true in a multithreaded scenario such as nodelets) */
  shopts.threadsafe      = false;
  shopts.autostart       = false;
  shopts.queue_size      = 5;
  shopts.transport_hints = ros::TransportHints();

  ROS_INFO("[%s]: creating subscribe handlers", ros::this_node::getName().c_str());

  /* name of the topic to be handled */
  const std::string topic_name = "/test_topic/pt";
  /* This is how a new SubscribeHandler object is initialized. */
  sh = mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>(shopts, topic_name, timeout_callback, message_callback);

  std::vector<ros::Timer> timers;
  for (int it = 0; it < 200; it++) {
    timers.push_back(nh.createTimer(ros::Duration(1e-3), 
    [&sh] ([[maybe_unused]] const ros::TimerEvent& evt)
    {
      if (sh.hasMsg()) {
        ROS_INFO_STREAM_THROTTLE(1.0, "time of last message: " << sh.lastMsgTime());
      }
    }
    ));
  }

  /* Type of the message may be accessed by C++11 decltype in case of need */
  ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>(topic_name, 5);

  /* Now let's just spin to process calbacks until the user decides to stop the program. */
  ros::Rate                   r(10);
  int                         n = 0;
  geometry_msgs::PointStamped msg;

  while (ros::ok() && n < 50) {

    if (n % 10 == 0) {
      ROS_INFO("[%s]: STARTING handler", ros::this_node::getName().c_str());
      started = true;
      sh.start();
    } else if (n % 5 == 0) {
      ROS_INFO("[%s]: STOPPING handler", ros::this_node::getName().c_str());
      started = false;
      sh.stop();
    }

    if (n % 4 == 0)
      msg.header.stamp = ros::Time::now() - ros::Duration(10.0);
    else
      msg.header.stamp = ros::Time::now();

    msg.point.x = msg.point.y = msg.point.z = n;

    /* msg.data = n; */
    pub.publish(msg);

    ROS_INFO_THROTTLE(1.0, "[%s]: Spinning", ros::this_node::getName().c_str());
    ros::spinOnce();

    r.sleep();
    n++;
  }

  result = got_messages_;

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "SubscribeHandlerTest");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
