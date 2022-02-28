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

    // from the launch file: intrinsic y = 1, p = 1.57, r = 2 => extrinsic r, p, y
    // we have not test this by checking the full rotation matrix, not by just comparing Euler Angles
    // that is because, there are more solutions to getRPY() than one, so comparing the resilts of
    // getIntrinsicRPY() might be erroneous

    Eigen::Vector3d x_vec_ref = AttitudeConverter(2, 1.57, 1, mrs_lib::RPY_EXTRINSIC).getVectorX();
    Eigen::Vector3d y_vec_ref = AttitudeConverter(2, 1.57, 1, mrs_lib::RPY_EXTRINSIC).getVectorY();
    Eigen::Vector3d z_vec_ref = AttitudeConverter(2, 1.57, 1, mrs_lib::RPY_EXTRINSIC).getVectorZ();

    Eigen::Vector3d x_vec = AttitudeConverter(tf_inv.getTransform().transform.rotation).getVectorX();
    Eigen::Vector3d y_vec = AttitudeConverter(tf_inv.getTransform().transform.rotation).getVectorY();
    Eigen::Vector3d z_vec = AttitudeConverter(tf_inv.getTransform().transform.rotation).getVectorZ();

    if (fabs(x_vec_ref[0] - x_vec[0]) > 1e-6 || fabs(x_vec_ref[1] - x_vec[1]) > 1e-6 || fabs(x_vec_ref[2] - x_vec[2]) > 1e-6) {
      printf("x vector's don't match\n");
      result *= 0;
    }

    if (fabs(y_vec_ref[0] - y_vec[0]) > 1e-6 || fabs(y_vec_ref[1] - y_vec[1]) > 1e-6 || fabs(y_vec_ref[2] - y_vec[2]) > 1e-6) {
      printf("y vector's don't match\n");
      result *= 0;
    }

    if (fabs(z_vec_ref[0] - z_vec[0]) > 1e-6 || fabs(z_vec_ref[1] - z_vec[1]) > 1e-6 || fabs(z_vec_ref[2] - z_vec[2]) > 1e-6) {
      printf("z vector's don't match\n");
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
//
/* TEST(TESTSuite, eigen_decomposed_test) //{ */

TEST(TESTSuite, eigen_decomposed_test) {

  ROS_INFO("[%s]: Testing the Eigen translation and rotation extraction", ros::this_node::getName().c_str());

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

    /* std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> test_vectors = {{{1,0,0},{0,-1,0}},{{0,1,0},{0,0,-1}},{{0,0,1},{1,0,0}}}; */

    auto [tran, rot] = tf.getTransformEigenDecomposed();

    Eigen::Vector3d tran_template = {1.0, 2.0, 3.0};
    Eigen::Vector3d vect_diff     = tran - tran_template;
    if (vect_diff.norm() > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: translation vector \
          [" << tran.transpose() << "]\
          does not match the expected value of \
          [" << tran_template.transpose() << "]");
      result *= 0;
    }

    Eigen::Matrix3d rot_mat_template;
    rot_mat_template << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    Eigen::Quaterniond rot_template(rot_mat_template);
    if (rot_template.angularDistance(rot) > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: rotation quaternion \
          [" << rot.x() << "," << rot.y() << ","
                                         << rot.z() << "," << rot.w() << "]\
          does not match the expected value of \
          [" << rot_template.x() << "," << rot_template.y()
                                         << "," << rot_template.z() << "," << rot_template.w() << "]");
      result *= 0;
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
