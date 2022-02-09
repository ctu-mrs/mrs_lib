#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/conversions.h>
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

  auto tfr = mrs_lib::Transformer("TransformerTest");

  std::optional<geometry_msgs::TransformStamped> tf_opt;

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

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    const auto tf_inv = Transformer::inverse(tf);
    std::cout << "from: " << Transformer::frame_from(tf_inv) << ", to: " << Transformer::frame_to(tf_inv) << ", stamp: " << tf_inv.header.stamp << std::endl;
    std::cout << tf_inv << std::endl;

    if (fabs(tf_inv.transform.translation.x - 1) > 1e-6 || fabs(tf_inv.transform.translation.y - 2) > 1e-6 ||
        fabs(tf_inv.transform.translation.z - 3) > 1e-6) {
      ROS_ERROR_THROTTLE(1.0, "[%s]: translation does not match", ros::this_node::getName().c_str());
      result *= 0;
    }

    auto [r, p, y] = mrs_lib::AttitudeConverter(tf_inv.transform.rotation).getIntrinsicRPY();

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

  auto tfr = mrs_lib::Transformer("TransformerTest");

  std::optional<geometry_msgs::TransformStamped> tf_opt;

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

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> test_vectors = {{{1, 0, 0}, {0, -1, 0}}, {{0, 1, 0}, {0, 0, -1}}, {{0, 0, 1}, {1, 0, 0}}};

    for (const auto& tv : test_vectors) {
      const auto rv = tfr.transform(tf, tv.first);
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

/* TEST(TESTSuite, mrs_reference_test) //{ */

TEST(TESTSuite, mrs_reference_test) {

  ROS_INFO("[%s]: Testing the mrs_msgs::ReferenceStamped transformation", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest");

  std::optional<geometry_msgs::TransformStamped> tf_opt;

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

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> test_vectors = {{{1, 0, 0}, {0, -1, 0}}, {{0, 1, 0}, {0, 0, -1}}, {{0, 0, 1}, {1, 0, 0}}};
    std::vector<std::pair<mrs_msgs::ReferenceStamped, mrs_msgs::ReferenceStamped>> test_refs;
    mrs_msgs::ReferenceStamped orig, tfd;
    orig.header.frame_id = "camera";
    orig.header.stamp = ros::Time::now();
    tfd.header.frame_id = "fcu";
    tfd.header.stamp = orig.header.stamp;
    for (const auto& vecs : test_vectors)
    {
      orig.reference.position.x = vecs.first.x();
      orig.reference.position.y = vecs.first.y();
      orig.reference.position.z = vecs.first.z();
      orig.reference.heading = std::atan2(vecs.first.y(), vecs.first.z());

      tfd.reference.position.x = vecs.second.x();
      tfd.reference.position.y = vecs.second.y();
      tfd.reference.position.z = vecs.second.z();
      tfd.reference.heading = std::atan2(vecs.second.y(), vecs.second.z());

      test_refs.push_back({orig, tfd});
    }

    for (const auto& tv : test_refs) {
      const auto rv_opt = tfr.transformSingle("fcu", tv.first);
      if (!rv_opt) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform reference [" << tv.first << "]");
        result *= 0;
      } else {
        const auto rv = rv_opt.value();
        const auto gt = tv.second;
        const Eigen::Vector3d vect_diff = mrs_lib::geometry::toEigen(rv.reference.position) - mrs_lib::geometry::toEigen(gt.reference.position);
        const double heading_diff = std::abs(rv.reference.heading - gt.reference.heading);
        if (vect_diff.norm() > 1e-6 || heading_diff > 1e-6)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed reference [" << tv.first << "] with value of ["
                                             << rv << "] does not match the expected value of [" << gt << "]");
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

/* TEST(TESTSuite, eigen_decomposed_test) //{ */

TEST(TESTSuite, eigen_decomposed_test) {

  ROS_INFO("[%s]: Testing the Eigen translation and rotation extraction", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest");

  std::optional<geometry_msgs::TransformStamped> tf_opt;

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

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    /* std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> test_vectors = {{{1,0,0},{0,-1,0}},{{0,1,0},{0,0,-1}},{{0,0,1},{1,0,0}}}; */

    const auto etf = tf2::transformToEigen(tf);
    const Eigen::Vector3d tran = etf.translation();
    const Eigen::Quaterniond rot(etf.rotation());

    Eigen::Vector3d tran_template = {1.0,2.0,3.0};
    Eigen::Vector3d vect_diff = tran - tran_template;
    if (vect_diff.norm() > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: translation vector \
          [" << tran.transpose() << "]\
          does not match the expected value of \
          [" << tran_template.transpose() << "]");
      result *= 0;
    }

    Eigen::Matrix3d rot_mat_template;
    rot_mat_template <<
        0.0,  0.0, 1.0, \
       -1.0,  0.0, 0.0, \
        0.0, -1.0, 0.0;
    Eigen::Quaterniond rot_template(rot_mat_template);
    if (rot_template.angularDistance(rot) > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: rotation quaternion \
          [" << rot.x() << "," << rot.y() << "," << rot.z() << "," << rot.w() << "]\
          does not match the expected value of \
          [" << rot_template.x() << "," << rot_template.y() << "," << rot_template.z() << "," << rot_template.w() << "]");
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
