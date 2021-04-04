#include "ros/duration.h"
#include "ros/spinner.h"
#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_lib/service_client_handler.h>

#include <string>
#include <thread>

#include <std_srvs/Trigger.h>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

ros::ServiceServer server;

mrs_lib::ServiceClientHandler<std_srvs::Trigger> client;
mrs_lib::ServiceClientHandler<std_srvs::Trigger> client2;

using namespace mrs_lib;

std::atomic<bool> service2_should_fail = true;

bool callbackService([[maybe_unused]] std_srvs::Trigger::Request& req, [[maybe_unused]] std_srvs::Trigger::Response& res) {

  res.message = "yes";
  res.success = true;

  ros::Duration(1.0).sleep();

  service2_should_fail = false;

  return true;
}

bool callbackFailedService([[maybe_unused]] std_srvs::Trigger::Request& req, [[maybe_unused]] std_srvs::Trigger::Response& res) {

  res.message = "yes";
  res.success = true;

  ros::Duration(0.2).sleep();

  if (service2_should_fail) {
    ROS_INFO("[%s]: simulating service failure", ros::this_node::getName().c_str());
    return false;
  } else {
    ROS_INFO("[%s]: service finally got through", ros::this_node::getName().c_str());
    return true;
  }
}

TEST(TESTSuite, main_test) {

  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // | ----------------- create a service server ---------------- |

  ros::ServiceServer server1 = nh.advertiseService("service", callbackService);
  ros::ServiceServer server2 = nh.advertiseService("service2", callbackFailedService);

  // | ----------------- create a service client ---------------- |

  client  = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "service");
  client2 = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "service2");

  ROS_INFO("[%s]: initialized", ros::this_node::getName().c_str());

  bool success = true;

  ros::AsyncSpinner spinner(10);
  spinner.start();

  std_srvs::Trigger srv;

  std::future<std_srvs::Trigger> future_res  = client.callAsync(srv, 3);
  std::future<std_srvs::Trigger> future2_res = client2.callAsync(srv, 10);

  client.call(srv);
  if (!srv.response.success) {
    success = false;
    ROS_ERROR("[%s]: normal service call failed", ros::this_node::getName().c_str());
  }

  future_res.wait();
  if (!future_res.get().response.success) {
    success = false;
    ROS_ERROR("[%s]: async service call failed", ros::this_node::getName().c_str());
  }

  future2_res.wait();
  if (!future2_res.get().response.success) {
    success = false;
    ROS_ERROR("[%s]: async failing service call failed", ros::this_node::getName().c_str());
  }

  ROS_INFO("[%s]: finished", ros::this_node::getName().c_str());

  EXPECT_TRUE(success);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "ServiceClientHandlerTest");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
