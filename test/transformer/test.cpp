#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <geometry_msgs/PointStamped.h>

#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace mrs_lib;
using namespace std;

/* TEST(TESTSuite, main_test) //{ */

TEST(TESTSuite, main_test) {

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest", "uav1");

  std::optional<mrs_lib::TransformStamped> tf_opt;

  // get the transform
  for (int it = 0; it < 1000 && ros::ok(); it++) {

    tf_opt = tfr.getTransform("local_origin", "fcu");

    if (tf_opt.has_value()) {
      break;
    }

    ros::Duration(0.1).sleep();
  }

  if (tf_opt.has_value()) {

    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    auto tf = tf_opt.value();
    std::cout << "from: " << tf.from() << ", to: " << tf.to() << ", stamp: " << tf.stamp() << std::endl;
    std::cout << tf.getTransform() << std::endl;

    auto tf_inv = tf.inverse();
    std::cout << "from: " << tf_inv.from() << ", to: " << tf_inv.to() << ", stamp: " << tf_inv.stamp() << std::endl;
    std::cout << tf_inv.getTransform() << std::endl;

    if (fabs(tf_inv.getTransform().transform.translation.x - 1) > 1e-6 || fabs(tf_inv.getTransform().transform.translation.y - 2) > 1e-6 ||
        fabs(tf_inv.getTransform().transform.translation.z - 3) > 1e-6) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: translation does not match", ros::this_node::getName().c_str());
      result *= 0;
    }

    auto [r, p, y] = mrs_lib::AttitudeConverter(tf_inv.getTransform().transform.rotation).getIntrinsicRPY();

    ROS_INFO("[%s]: r: %.2f", ros::this_node::getName().c_str(), r);
    ROS_INFO("[%s]: p: %.2f", ros::this_node::getName().c_str(), p);
    ROS_INFO("[%s]: y: %.2f", ros::this_node::getName().c_str(), y);

    if (fabs(r - 1) > 1e-6 || fabs(p - 1.57) > 1e-6 || fabs(y - 2) > 1e-6) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: rotation does not match", ros::this_node::getName().c_str());
      result *= 0;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: missing tf", ros::this_node::getName().c_str());
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, eigen_vector3d_test) //{ */

TEST(TESTSuite, eigen_vector3d_test) {

  ROS_INFO("[%s]: Testing the Eigen::Vector3d transformation", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest", "uav1");

  std::optional<mrs_lib::TransformStamped> tf_opt;

  // get the transform
  for (int it = 0; it < 1000 && ros::ok(); it++) {

    tf_opt = tfr.getTransform("camera", "fcu");

    if (tf_opt.has_value()) {
      break;
    }

    ros::Duration(0.1).sleep();
  }

  if (tf_opt.has_value()) {

    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    auto tf = tf_opt.value();
    std::cout << "from: " << tf.from() << ", to: " << tf.to() << ", stamp: " << tf.stamp() << std::endl;
    std::cout << tf.getTransform() << std::endl;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> test_vectors = {{{1, 0, 0}, {0, -1, 0}}, {{0, 1, 0}, {0, 0, -1}}, {{0, 0, 1}, {1, 0, 0}}};

    for (auto& tv : test_vectors) {
      auto rv = tfr.transformHeaderless(tf, tv.first);
      if (!rv) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.first.transpose() << "]");
        result *= 0;
      } else {
        Eigen::Vector3d vect_diff = rv.value() - tv.second;
        if (vect_diff.norm() > 1e-6) {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: rotated vector [" << tv.first.transpose() << "] with value of ["
                                             << rv.value().transpose() << "] does not match the expected value of [" << tv.second.transpose() << "]");
          result *= 0;
        }
      }
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[%s]: missing tf", ros::this_node::getName().c_str());
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  // Set up ROS.
  ros::init(argc, argv, "transformer_tests");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
