#include "ros/init.h"
#include <ros/node_handle.h>
#include <mrs_lib/service_client_handler.h>

#include <string>
#include <ros/ros.h>

#include <thread>

#include <std_srvs/Trigger.h>

ros::ServiceServer                               server;
mrs_lib::ServiceClientHandler<std_srvs::Trigger> client;

bool callbackService([[maybe_unused]] std_srvs::Trigger::Request& req, [[maybe_unused]] std_srvs::Trigger::Response& res) {

  ROS_INFO("[%s]: service call received", ros::this_node::getName().c_str());

  return true;
}

void threadMain(void) {

  while (ros::ok()) {

    ROS_INFO_ONCE("[%s]: thread spinning", ros::this_node::getName().c_str());

    std_srvs::Trigger srv;
    client.call(srv);
  }
}

int main(int argc, char** argv) {

  const std::string node_name("service_client_handler_example");

  ros::init(argc, argv, node_name);
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // | ----------------- create a service server ---------------- |

  ros::ServiceServer server = nh.advertiseService("service", callbackService);

  // | ----------------- create a service client ---------------- |

  client = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "service");

  // | ------------------------- thread ------------------------- |

  std::thread thread_main = std::thread(threadMain);

  ROS_INFO("[%s]: initialized", ros::this_node::getName().c_str());

  ros::Rate rate(100.0);

  while (ros::ok()) {

    ROS_INFO_ONCE("[%s]: spinning", ros::this_node::getName().c_str());

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

  ros::shutdown();

  return 0;
}
