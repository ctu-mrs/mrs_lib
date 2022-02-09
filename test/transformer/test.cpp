#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/conversions.h>
#include <mrs_lib/attitude_converter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_eigen/tf2_eigen.h>

#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace mrs_lib;
using namespace std;
using namespace mrs_lib::geometry;

Eigen::Affine3d local2local;
Eigen::Affine3d local2fcu;
Eigen::Affine3d local2utm;
Eigen::Affine3d fcu2cam;
std::unique_ptr<tf2_ros::TransformBroadcaster> bc;

void publish_transforms(const ros::Time& t = ros::Time::now())
{
  geometry_msgs::TransformStamped tf;

  tf = tf2::eigenToTransform(local2local);
  tf.header.frame_id = "uav3/local_origin";
  tf.header.stamp = t;
  tf.child_frame_id = "uav66/local_origin";
  bc->sendTransform(tf);
  ros::spinOnce();

  tf = tf2::eigenToTransform(local2fcu);
  tf.header.frame_id = "uav66/local_origin";
  tf.header.stamp = t;
  tf.child_frame_id = "uav66/fcu";
  bc->sendTransform(tf);
  ros::spinOnce();

  tf = tf2::eigenToTransform(local2utm);
  tf.header.frame_id = "uav66/local_origin";
  tf.header.stamp = t;
  tf.child_frame_id = "uav66/utm_origin";
  bc->sendTransform(tf);
  ros::spinOnce();

  tf = tf2::eigenToTransform(fcu2cam);
  tf.header.frame_id = "uav66/fcu";
  tf.header.stamp = t;
  tf.child_frame_id = "uav66/camera";
  bc->sendTransform(tf);
  ros::spinOnce();
}

std::optional<geometry_msgs::TransformStamped> wait_for_tf(const std::string& from, const std::string& to, mrs_lib::Transformer& tfr, const ros::Time& publish_t = ros::Time::now(), const ros::Time& stamp = ros::Time(0))
{
  std::cout << "Waiting for transformation from " << from << " to " << to << ".\n";
  std::optional<geometry_msgs::TransformStamped> tf_opt;
  // get the transform
  for (int it = 0; it < 20 && ros::ok(); it++)
  {
    publish_transforms(publish_t);
    ros::spinOnce();
    tf_opt = tfr.getTransform(from, to, stamp);

    if (tf_opt.has_value())
      break;

    ros::Duration(0.1).sleep();
  }
  return tf_opt;
}

/* TEST(TESTSuite, main_test) //{ */

TEST(TESTSuite, main_test)
{
  std::cout << "Running main_test\n";

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest");
  tfr.setDefaultPrefix("uav66");

  const std::string from = "fcu";
  const std::string to = "local_origin";
  const auto tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    const auto tf_inv = Transformer::inverse(tf);
    std::cout << "from: " << Transformer::frame_from(tf_inv) << ", to: " << Transformer::frame_to(tf_inv) << ", stamp: " << tf_inv.header.stamp << std::endl;
    std::cout << tf_inv << std::endl;

    if (fabs(tf_inv.transform.translation.x - local2fcu.translation().x()) > 1e-6 || fabs(tf_inv.transform.translation.y - local2fcu.translation().y()) > 1e-6 ||
        fabs(tf_inv.transform.translation.z - local2fcu.translation().z()) > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "translation does not match (gt: " << local2fcu.translation().transpose() << ")");
      result *= 0;
    }

    const Eigen::Isometry3d etf = tf2::transformToEigen(tf_inv);
    const quat_t etf_rot(etf.rotation());
    const double angle_diff = etf_rot.angularDistance(quat_t(local2fcu.rotation()));

    if (angle_diff > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "translation does not match (by angle: " << angle_diff << ")");
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
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from = "uav66/camera";
  const std::string to = "fcu";
  const auto tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    std::vector<Eigen::Vector3d> test_vectors = {{1, 0, 0}, {0, -1, 666}, {0, 0, 0}, {0, 1.1, -1}, {0, 0, 18}, {3, 0, 0}};

    for (const auto& tv : test_vectors) {
      const auto rv = tfr.transform(tf, tv);
      if (!rv)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.transpose() << "]");
        result *= 0;
      }
      else
      {
        const vec3_t gt = fcu2cam.inverse()*tv;
        const vec3_t vect_diff = rv.value() - gt;
        if (vect_diff.norm() > 1e-6)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: rotated vector [" << gt.transpose() << "] with value of ["
                                             << rv.value().transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
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
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from = "uav66/fcu";
  const std::string to = "camera";
  const auto tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    const std::vector<Eigen::Vector3d> test_vectors = {{1, 0, 0}, {0, -1, 666}, {0, 0, 0}, {0, 1.1, -1}, {0, 0, 18}, {3, 0, 0}};
    std::vector<std::pair<mrs_msgs::ReferenceStamped, mrs_msgs::ReferenceStamped>> test_refs;
    mrs_msgs::ReferenceStamped orig, tfd;
    orig.header.frame_id = "camera";
    orig.header.stamp = ros::Time::now();
    tfd.header.frame_id = "fcu";
    tfd.header.stamp = orig.header.stamp;
    for (const auto& vec : test_vectors)
    {
      orig.reference.position.x = vec.x();
      orig.reference.position.y = vec.y();
      orig.reference.position.z = vec.z();
      orig.reference.heading = std::atan2(vec.y(), vec.z());

      const vec3_t gt = fcu2cam*vec;
      tfd.reference.position.x = gt.x();
      tfd.reference.position.y = gt.y();
      tfd.reference.position.z = gt.z();
      tfd.reference.heading = headingFromRot(fcu2cam.rotation()*quaternionFromHeading(orig.reference.heading));

      test_refs.push_back({orig, tfd});
    }

    for (const auto& tv : test_refs) {
      const auto rv_opt = tfr.transform(tf, tv.first);
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

/* TEST(TESTSuite, quaternion_test) //{ */

TEST(TESTSuite, quaternion_test)
{

  ROS_INFO("[%s]: Testing the geometry_msgs::Quaternion transformation", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest");
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from = "local_origin";
  const std::string to = "fcu";
  const auto tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value())
  {
    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    std::vector<geometry_msgs::Quaternion> tsts;
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));
    tsts.push_back(tf2::toMsg(quat_t::UnitRandom()));

    for (const auto& tv : tsts)
    {
      const auto rv = tfr.transform(tf, tv);
      if (!rv)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform quaternion [" << tv << "]");
        result *= 0;
      }
      else
      {
        quat_t orig, tfd;
        tf2::fromMsg(tv, orig);
        tf2::fromMsg(rv.value(), tfd);
        const quat_t gt(local2fcu.rotation()*orig);
        const double angle_diff = gt.angularDistance(tfd);
        if (angle_diff > 1e-6)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: rotated quaternion [" << tv << "] with value of ["
                                             << rv.value() << "] does not match the expected value of [" << tf2::toMsg(gt) << "]");
          result *= 0;
        }
      }
    }

  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: missing tf", ros::this_node::getName().c_str());
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, latlon_test) //{ */

TEST(TESTSuite, latlon_test)
{

  ROS_INFO("[%s]: Testing transformation from/to latlon", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest");
  tfr.setDefaultPrefix("uav66");
  tfr.setLatLon(0, 0);

  publish_transforms();

  const std::string from = "local_origin";
  const std::string to = mrs_lib::LATLON_ORIGIN;
  const auto tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value())
  {
    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    geometry_msgs::Point tv;
    tv.x = 5;
    tv.y = 6;
    tv.z = 7;
    const auto rv = tfr.transform(tf, tv);
    if (!rv)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv << "]");
      result *= 0;
    }
    else
    {
    }
  }
  else
  {
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
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const std::string from = "camera";
  const std::string to = "fcu";
  const auto tf_opt = wait_for_tf(from, to, tfr);

  if (tf_opt.has_value()) {

    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    /* std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> test_vectors = {{{1,0,0},{0,-1,0}},{{0,1,0},{0,0,-1}},{{0,0,1},{1,0,0}}}; */

    const auto etf = tf2::transformToEigen(tf);
    const Eigen::Vector3d tran = etf.translation();
    const Eigen::Quaterniond rot(etf.rotation());

    const vec3_t gt = fcu2cam.inverse().translation();
    const vec3_t vect_diff = tran - gt;
    if (vect_diff.norm() > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: translation vector \
          [" << tran.transpose() << "]\
          does not match the expected value of \
          [" << gt.transpose() << "]");
      result *= 0;
    }

    const quat_t rot_template(fcu2cam.inverse().rotation());
    if (rot_template.angularDistance(rot) > 1e-6)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: rotation quaternion \
          [" << rot.x() << "," << rot.y() << "," << rot.z() << "," << rot.w() << "]\
          does not match the expected value of \
          [" << rot_template.x() << "," << rot_template.y() << "," << rot_template.z() << "," << rot_template.w() << "]");
      result *= 0;
    }

  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: missing tf", ros::this_node::getName().c_str());
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, retry_latest_test) //{ */

TEST(TESTSuite, retry_latest_test) {

  ROS_INFO("[%s]: Testing the functionality of retrying with latest time in case of failure", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest");
  tfr.setDefaultPrefix("uav66");

  const ros::Time t = ros::Time::now();
  publish_transforms(t);

  const std::string from = "camera";
  const std::string to = "local_origin";
  auto tf_opt = wait_for_tf(from, to, tfr, t, t - ros::Duration(1000));

  if (tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tfr.retryLookupNewest(true);
  tf_opt = wait_for_tf(from, to, tfr, t, t - ros::Duration(1000));

  if (!tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "didn't get the transformation!");
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, prefix_test) //{ */

TEST(TESTSuite, prefix_test)
{

  ROS_INFO("[%s]: Testing the functionality of retrying with latest time in case of failure", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("TransformerTest");

  const ros::Time t = ros::Time::now();
  publish_transforms(t);

  auto tf_opt = wait_for_tf("camera", "local_origin", tfr);
  if (tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("uav3/local_origin", "uav66/local_origin", tfr);
  if (!tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "didn't get the transformation!");
    result *= 0;
  }

  tfr.setDefaultPrefix("uav66");
  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("camera", "local_origin", tfr);
  if (!tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "didn't get the transformation!");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("uav66/camera", "uav66/local_origin", tfr);
  if (!tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "didn't get the transformation!");
    result *= 0;
  }

  tfr.setDefaultPrefix("");
  tf_opt = std::nullopt;
  tf_opt = wait_for_tf("camera", "uav66/local_origin", tfr);
  if (tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "got the transform (that should not happen)");
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  // Set up ROS.
  ros::init(argc, argv, "transformer_tests");
  ros::NodeHandle nh = ros::NodeHandle("~");
  bc = std::make_unique<tf2_ros::TransformBroadcaster>();

  local2local = anax_t(3.2, vec3_t::UnitZ());
  local2local.translation() = 10*vec3_t::Random();
  local2fcu = anax_t(1.2, vec3_t::UnitZ()) * anax_t(0.2, vec3_t::UnitX());
  local2fcu.translation() = vec3_t::Random();
  local2utm = Eigen::Affine3d::Identity();
  fcu2cam = anax_t(1.54, vec3_t::UnitZ()) * anax_t(1.54, vec3_t::UnitX()) * anax_t(1.54, vec3_t::UnitY());
  fcu2cam.translation() = vec3_t(0.2, 0, 0);

  ros::Time::waitForValid();

  std::cout << "Initialized, running tests\n";

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
