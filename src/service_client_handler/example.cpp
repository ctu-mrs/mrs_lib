#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_lib/service_client_handler.h>

#include <string>
#include <thread>

#include <std_srvs/Trigger.h>

ros::ServiceServer server1;
ros::ServiceServer server2;

mrs_lib::ServiceClientHandler<std_srvs::Trigger> client1;
mrs_lib::ServiceClientHandler<std_srvs::Trigger> client2;

int counter_1 = 0;
int counter_2 = 0;

bool callbackService1([[maybe_unused]] std_srvs::Trigger::Request& req, [[maybe_unused]] std_srvs::Trigger::Response& res) {

  ROS_INFO("[%s]: service call 1 received", ros::this_node::getName().c_str());

  ros::Duration(1.0).sleep();

  ROS_INFO("[%s]: service 1 call stopped", ros::this_node::getName().c_str());

  ROS_INFO("[%s]: service 1 counter: %d", ros::this_node::getName().c_str(), counter_1++);

  return true;
}

bool callbackService2([[maybe_unused]] std_srvs::Trigger::Request& req, [[maybe_unused]] std_srvs::Trigger::Response& res) {

  ROS_INFO("[%s]: service call 2 received", ros::this_node::getName().c_str());

  ros::Duration(1.0).sleep();

  ROS_INFO("[%s]: service 2 call stopped", ros::this_node::getName().c_str());

  ROS_INFO("[%s]: service 2 counter: %d", ros::this_node::getName().c_str(), counter_2++);

  return true;
}

void threadMain(void) {

  while (ros::ok()) {

    ROS_INFO_ONCE("[%s]: thread spinning", ros::this_node::getName().c_str());

    std_srvs::Trigger srv;

    std::future<std_srvs::Trigger> res = client1.callAsync(srv, 10);
    client2.call(srv, 10, 0.1);
  }
}

int main(int argc, char** argv) {

  const std::string node_name("service_client_handler_example");

  ros::init(argc, argv, node_name);
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // | ----------------- create a service server ---------------- |

  ros::ServiceServer server1 = nh.advertiseService("service1", callbackService1);
  ros::ServiceServer server2 = nh.advertiseService("service2", callbackService2);

  // | ----------------- create a service client ---------------- |

  client1 = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "service1");
  client2.initialize(nh, "service2");

  // | ------------------------- thread ------------------------- |

  std::thread thread_main = std::thread(threadMain);

  ROS_INFO("[%s]: initialized", ros::this_node::getName().c_str());

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

  ros::shutdown();

  return 0;
}
