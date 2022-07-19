#include <mrs_lib/transformer.h>
#include <mrs_lib/gps_conversions.h>
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

//{ publish_transform helper function
void publish_transforms(const ros::Time& t = ros::Time::now())
{
  geometry_msgs::TransformStamped tf;

  tf = tf2::eigenToTransform(local2local);
  tf.child_frame_id = "uav3/local_origin";
  tf.header.frame_id = "uav66/local_origin";
  tf.header.stamp = t;
  bc->sendTransform(tf);
  ros::spinOnce();

  tf = tf2::eigenToTransform(local2fcu);
  tf.child_frame_id = "uav66/local_origin";
  tf.header.frame_id = "uav66/fcu";
  tf.header.stamp = t;
  bc->sendTransform(tf);
  ros::spinOnce();

  tf = tf2::eigenToTransform(local2utm.inverse());
  tf.child_frame_id = "uav66/utm_origin";
  tf.header.frame_id = "uav66/local_origin";
  tf.header.stamp = t;
  bc->sendTransform(tf);
  ros::spinOnce();

  tf = tf2::eigenToTransform(fcu2cam);
  tf.child_frame_id = "uav66/fcu";
  tf.header.frame_id = "uav66/camera";
  tf.header.stamp = t;
  bc->sendTransform(tf);
  ros::spinOnce();
}
//}

/* //{ wait_for_tf helper function*/
std::optional<geometry_msgs::TransformStamped> wait_for_tf(const std::string& from, const std::string& to, mrs_lib::Transformer& tfr, const ros::Time& publish_t = ros::Time::now(), const ros::Time& stamp = ros::Time(0))
{
  std::cout << "Waiting for transformation from " << from << " to " << to << ".\n";
  std::optional<geometry_msgs::TransformStamped> tf_opt;
  // get the transform
  for (int it = 0; it < 10 && ros::ok(); it++)
  {
    publish_transforms(publish_t);
    ros::spinOnce();
    tf_opt = tfr.getTransform(from, to, stamp);

    if (tf_opt.has_value())
      break;

    ros::Duration(0.05).sleep();
  }
  return tf_opt;
}
//}

/* TEST(TESTSuite, main_test) //{ */

TEST(TESTSuite, main_test)
{
  std::cout << "Running main_test\n";

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_main_test");
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

/* TEST(TESTSuite, tf_times_test) //{ */

TEST(TESTSuite, tf_times_test)
{
  std::cout << "Running tf_times_test\n";

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_tf_times_test");

  const ros::Time t1 = ros::Time(1);
  const std::string from = "uav1/fcu";
  const ros::Time t2 = ros::Time(2);
  const std::string to = "uav2/fcu";
  const std::string fixed = "common_origin";

  // create and publish the transformations
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = fixed;
  tf.transform.rotation.w = 1;
  Eigen::Isometry3d common2uav1, common2uav2;

  // UAV1 is at position [-1,0,2] at time t1
  tf.header.stamp = t1;
  tf.child_frame_id = from;
  common2uav1.translation() = vec3_t(-1, 0, 2);
  tf.transform.translation = mrs_lib::geometry::fromEigenVec(common2uav1.translation());
  bc->sendTransform(tf);
  ros::spinOnce();

  // UAV2 is at position [-10,0,3] at time t1
  tf.header.stamp = t1;
  tf.child_frame_id = to;
  common2uav2.translation() = vec3_t(-10, 0, 3);
  tf.transform.translation = mrs_lib::geometry::fromEigenVec(common2uav2.translation());
  bc->sendTransform(tf);
  ros::spinOnce();

  // UAV1 is at position [3,0,2] at time t2
  tf.header.stamp = t2;
  tf.child_frame_id = from;
  common2uav1.translation() = vec3_t(3, 0, 2);
  tf.transform.translation = mrs_lib::geometry::fromEigenVec(common2uav1.translation());
  bc->sendTransform(tf);
  ros::spinOnce();

  // UAV2 is at position [60,0,3] at time t2
  tf.header.stamp = t2;
  tf.child_frame_id = to;
  common2uav2.translation() = vec3_t(60, 0, 3);
  tf.transform.translation = mrs_lib::geometry::fromEigenVec(common2uav2.translation());
  bc->sendTransform(tf);
  ros::spinOnce();

  std::cout << "Waiting for transformation from " << from << " to " << to << " through " << fixed << ".\n";
  tfr.setLookupTimeout(ros::Duration(2));
  // where was UAV1 at time t1 relative to position of UAV2 at time t2 assuming common_frame is fixed??
  const std::optional<geometry_msgs::TransformStamped> tf_opt = tfr.getTransform(from, t1, to, t2, fixed);

  if (tf_opt.has_value())
  {
    ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

    const auto tf = tf_opt.value();
    std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
    std::cout << tf << std::endl;

    // the expected transformation is a translation [-61, 0, -1]
    const vec3_t tr = toEigen(tf.transform.translation);
    if (fabs(tr.x() - -61) > 1e-6 || fabs(tr.y() - 0) > 1e-6 || fabs(tr.z() - -1) > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "translation does not match (gt: -61, 0, -1), actual: " << tr.transpose() << ")");
      result *= 0;
    }

    const Eigen::Isometry3d etf = tf2::transformToEigen(tf);
    const quat_t etf_rot(etf.rotation());
    const double angle_diff = etf_rot.angularDistance(quat_t::Identity());

    if (angle_diff > 1e-6) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "rotation does not match (by angle: " << angle_diff << ")");
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

TEST(TESTSuite, eigen_vector3d_test)
{

  ROS_INFO("[%s]: Testing the Eigen::Vector3d transformation", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_eigen_vector3d_test");
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
      // | ------------- normal transform (as a vector) ------------- |
      {
        const auto rv = tfr.transform(tv, tf);
        if (!rv)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        }
        else
        {
          // transform it as a *vector*, not as a point!
          const vec3_t gt = fcu2cam.inverse().rotation()*tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6)
          {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed vector [" << gt.transpose() << "] with value of ["
                                               << rv.value().transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a vector ---------------------- |
      {
        const auto rv = tfr.transformAsVector(tv, tf);
        if (!rv)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        }
        else
        {
          // transform it as a *vector*, not as a point!
          const vec3_t gt = fcu2cam.inverse().rotation()*tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6)
          {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed vector [" << gt.transpose() << "] with value of ["
                                               << rv.value().transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a point ----------------------- |
      {
        const auto rv = tfr.transformAsPoint(tv, tf);
        if (!rv)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        }
        else
        {
          // transform it as a *vector*, not as a point!
          const vec3_t gt = fcu2cam.inverse()*tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6)
          {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed vector [" << gt.transpose() << "] with value of ["
                                               << rv.value().transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a vector ---------------------- |
      {
        const auto rv = tfr.transformAsVector(mrs_lib::Transformer::frame_from(tf), tv, mrs_lib::Transformer::frame_to(tf), tf.header.stamp);
        if (!rv)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        }
        else
        {
          // transform it as a *vector*, not as a point!
          const vec3_t gt = fcu2cam.inverse().rotation()*tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6)
          {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed vector [" << gt.transpose() << "] with value of ["
                                               << rv.value().transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
        }
      }

      // | ----------------------- as a point ----------------------- |
      {
        const auto rv = tfr.transformAsPoint(mrs_lib::Transformer::frame_from(tf), tv, mrs_lib::Transformer::frame_to(tf), tf.header.stamp);
        if (!rv)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.transpose() << "]");
          result *= 0;
        }
        else
        {
          // transform it as a *vector*, not as a point!
          const vec3_t gt = fcu2cam.inverse()*tv;
          const vec3_t vect_diff = rv.value() - gt;
          if (vect_diff.norm() > 1e-6)
          {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed vector [" << gt.transpose() << "] with value of ["
                                               << rv.value().transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
            result *= 0;
          }
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

/* TEST(TESTSuite, transform_single) //{ */

TEST(TESTSuite, transform_single)
{

  ROS_INFO("[%s]: Testing transformSingle", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_transform_single");
  tfr.setDefaultPrefix("uav66");

  const ros::Time t = ros::Time::now();
  publish_transforms(t);

  const std::string from = "uav66/camera";
  const std::string to = "fcu";
  wait_for_tf(from, to, tfr, t);

  // test transforming a geometry_msgs::PointStamped message
  {
    geometry_msgs::PointStamped::Ptr pt = boost::make_shared<geometry_msgs::PointStamped>();
    pt->point.x = 666;
    pt->point.y = 667;
    pt->point.z = 668;
    pt->header.frame_id = from;
    pt->header.stamp = t;
    const vec3_t tv(pt->point.x, pt->point.y, pt->point.z);
    
    const auto rv_opt = tfr.transformSingle(pt, to);
    if (!rv_opt)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.transpose() << "]");
      result *= 0;
    }
    else
    {
      const vec3_t rv(rv_opt.value()->point.x, rv_opt.value()->point.y, rv_opt.value()->point.z);
      const vec3_t gt = fcu2cam.inverse()*tv;
      const vec3_t vect_diff = rv - gt;
      if (vect_diff.norm() > 1e-6)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed vector [" << gt.transpose() << "] with value of ["
                                           << rv.transpose() << "] does not match the expected value of [" << gt.transpose() << "]");
        result *= 0;
      }
    }
  }

  // test transforming a headerless message
  {
    const quat_t tv( anax_t(2, vec3_t::UnitZ()) * anax_t(-0, vec3_t::UnitX()) );
    geometry_msgs::Quaternion::Ptr q = boost::make_shared<geometry_msgs::Quaternion>();
    q->x = tv.x();
    q->y = tv.y();
    q->z = tv.z();
    q->w = tv.w();
    geometry_msgs::Quaternion::ConstPtr cq = q;
    
    const auto rv_opt = tfr.transformSingle(from, cq, to, t);
    if (!rv_opt)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform vector [" << tv.vec() << ", " << tv.w() << "]");
      result *= 0;
    }
    else
    {
      const quat_t rv(rv_opt.value()->w, rv_opt.value()->x, rv_opt.value()->y, rv_opt.value()->z);
      const quat_t gt(fcu2cam.inverse().rotation()*tv);
      const double ang_diff = rv.angularDistance(gt);
      if (ang_diff > 1e-6)
      {
        ROS_ERROR_STREAM("Transformed quaternion doesn't match (angular difference is " << ang_diff << ")");
        result *= 0;
      }
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, mrs_reference_test) //{ */

TEST(TESTSuite, mrs_reference_test)
{

  ROS_INFO("[%s]: Testing the mrs_msgs::ReferenceStamped transformation", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_mrs_reference_test");
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

    const std::vector<Eigen::Vector3d> test_points = {{1, 0, 0}, {0, -1, 666}, {0, 0, 0}, {0, 1.1, -1}, {0, 0, 18}, {3, 0, 0}};
    std::vector<std::pair<mrs_msgs::ReferenceStamped, mrs_msgs::ReferenceStamped>> test_refs;
    mrs_msgs::ReferenceStamped orig, tfd;
    orig.header.frame_id = "camera";
    orig.header.stamp = ros::Time::now();
    tfd.header.frame_id = "fcu";
    tfd.header.stamp = orig.header.stamp;
    for (const auto& vec : test_points)
    {
      orig.reference.position.x = vec.x();
      orig.reference.position.y = vec.y();
      orig.reference.position.z = vec.z();
      orig.reference.heading = std::atan2(vec.y(), vec.z());

      // transform it as a *point*, not as a vector!
      const vec3_t gt = fcu2cam*vec;
      tfd.reference.position.x = gt.x();
      tfd.reference.position.y = gt.y();
      tfd.reference.position.z = gt.z();
      tfd.reference.heading = headingFromRot(fcu2cam.rotation()*quaternionFromHeading(orig.reference.heading));

      test_refs.push_back({orig, tfd});
    }

    for (const auto& tv : test_refs) {
      const auto rv_opt = tfr.transform(tv.first, tf);
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

  auto tfr = mrs_lib::Transformer("Transformer_quaternion_test");
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
      const auto rv = tfr.transform(tv, tf);
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
          ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: transformed quaternion [" << tv << "] with value of ["
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

/* TEST(TESTSuite, opencv_test) //{ */

TEST(TESTSuite, opencv_test)
{

  ROS_INFO("[%s]: Testing the cv::Point3<T> transformation", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_opencv_test");
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

    const cv::Point3d tv(1,2,3);
    const auto rv = tfr.transform(tv, tf);
    if (!rv)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << ros::this_node::getName() << "]: Failed to transform point [" << tv.x << ", " << tv.y << ", " << tv.z << "]");
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

/* TEST(TESTSuite, latlon_test) //{ */

bool compare_gt_latlon(const geometry_msgs::Point& tv, const geometry_msgs::Point& rv, const char utm_zone[10], const bool ll2local)
{
  vec3_t gt;
  if (ll2local)
  {
    // convert LAT-LON to UTM
    Eigen::Vector3d utm;
    mrs_lib::UTM(tv.x, tv.y, &(utm.x()), &(utm.y()));
    // copy the height from the input
    utm.z() = tv.z;
    const vec3_t local = local2utm.inverse()*utm;
    gt = local;
  }
  else
  {
    const vec3_t utm = local2utm*mrs_lib::geometry::toEigen(tv);
    Eigen::Vector3d latlon;
    mrs_lib::UTMtoLL(utm.y(), utm.x(), utm_zone, latlon.x(), latlon.y());
    latlon.z() = utm.z();
    gt = latlon;
  }

  const Eigen::Vector3d vect_diff = mrs_lib::geometry::toEigen(rv) - gt;
  if (vect_diff.norm() > 1e-6)
  {
    ROS_ERROR_STREAM("<< Transformed [" << tv << "] with value of ["
                                       << rv << "] does not match the expected value of [" << gt.transpose() << "]");
    return false;
  }

  std::cout << "<< Expected success happened\n";
  return true;
}

bool compare_gt_latlon([[maybe_unused]] const quat_t& tv, [[maybe_unused]] const quat_t& rv, [[maybe_unused]] const char utm_zone[10], [[maybe_unused]] const bool ll2local)
{
  ROS_ERROR_STREAM("this should never happen");
  return false;
}

template <typename T>
bool trytransform_latlon(const T& tv, mrs_lib::Transformer& tfr, const char utm_zone[10], const bool ll2local, const bool expect_ok)
{
  const std::string local = "local_origin";
  const std::string ll = mrs_lib::LATLON_ORIGIN;
  const auto tf_opt = ll2local ? wait_for_tf(ll, local, tfr) : wait_for_tf(local, ll, tfr);

  if (!tf_opt.has_value())
  {
    ROS_ERROR("[%s]: missing tf", ros::this_node::getName().c_str());
    return false;
  }

  ROS_INFO("[%s]: got the transform", ros::this_node::getName().c_str());

  const auto tf = tf_opt.value();
  std::cout << "from: " << Transformer::frame_from(tf) << ", to: " << Transformer::frame_to(tf) << ", stamp: " << tf.header.stamp << std::endl;
  std::cout << tf << std::endl;

  const auto rv_opt = tfr.transform(tv, tf);
  if (!rv_opt)
  {
    if (!expect_ok)
    {
      std::cout << "<< Expected failure happened\n";
      return true;
    }

    ROS_ERROR_STREAM("<< Failed to transform message");
    return false;
  }

  if (rv_opt && !expect_ok)
  {
    ROS_ERROR_STREAM("<< Transformed message - that should not be possible!");
    return false;
  }

  const auto rv = rv_opt.value();
  return compare_gt_latlon(tv, rv, utm_zone, ll2local);
}

TEST(TESTSuite, latlon_test)
{

  ROS_INFO("[%s]: Testing transformation from/to latlon", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_latlon_test");
  tfr.setDefaultPrefix("uav66");

  publish_transforms();

  const double lat = 1;
  const double lon = 2;
  double utm_x, utm_y;
  char utm_zone_[10];
  mrs_lib::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_);

  geometry_msgs::Point tv;
  tv.x = 5;
  tv.y = 6;
  tv.z = 7;
  const quat_t tq(1, 0, 0, 0);
  // try transforming a vector from latlon to local frame and expect a failure
  ROS_INFO("[%s]: Testing transformation of vector from local to latlon, expecting failure >>", ros::this_node::getName().c_str());
  result = result && trytransform_latlon(tv, tfr, utm_zone_, false, false);

  // try transforming a vector from local to latlon frame and expect everything OK
  ROS_INFO("[%s]: Testing transformation of vector from latlon to local, expecting all OK >>", ros::this_node::getName().c_str());
  result = result && trytransform_latlon(tv, tfr, utm_zone_, true, true);

  // try transforming a quaternion from local to latlon frame and expect failure because there's no UTMtoLL method for quaternion
  ROS_INFO("[%s]: Testing transformation of quaternion from latlon to local, expecting failure >>", ros::this_node::getName().c_str());
  result = result && trytransform_latlon(tq, tfr, utm_zone_, true, false);

  // after latlon is set, lookup from utm to latlon should be OK
  ROS_INFO("[%s]: Setting lattitude and longitude to Transformer, UTM->latlon should be fine now.", ros::this_node::getName().c_str());
  tfr.setLatLon(1, 2);

  // try transforming a vector from latlon to local frame and expect everything OK
  ROS_INFO("[%s]: Testing transformation of vector from local to latlon, expecting all OK >>", ros::this_node::getName().c_str());
  result = result && trytransform_latlon(tv, tfr, utm_zone_, false, true);

  // try transforming a vector from local to latlon frame and expect everything OK
  ROS_INFO("[%s]: Testing transformation of vector from latlon to local, expecting all OK >>", ros::this_node::getName().c_str());
  result = result && trytransform_latlon(tv, tfr, utm_zone_, true, true);

  // try transforming a quaternion from latlon to local frame and expect failure because there's no UTMtoLL method for quaternion
  ROS_INFO("[%s]: Testing transformation of quaternion from local to latlon, expecting failure >>", ros::this_node::getName().c_str());
  result = result && trytransform_latlon(tq, tfr, utm_zone_, false, false);

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, eigen_decomposed_test) //{ */

TEST(TESTSuite, eigen_decomposed_test)
{

  ROS_INFO("[%s]: Testing the Eigen translation and rotation extraction", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_eigen_decomposed_test");
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
          [" << rot.x() << "," << rot.y() << ","
                                         << rot.z() << "," << rot.w() << "]\
          does not match the expected value of \
          [" << rot_template.x() << "," << rot_template.y()
                                         << "," << rot_template.z() << "," << rot_template.w() << "]");
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

TEST(TESTSuite, retry_latest_test)
{

  ROS_INFO("[%s]: Testing the functionality of retrying with latest time in case of failure", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_retry_latest_test");
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

  ROS_INFO("[%s]: Testing the functionality of default prefix", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_prefix_test");

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

/* TEST(TESTSuite, empty_frame_test) //{ */

TEST(TESTSuite, default_frame_test)
{

  ROS_INFO("[%s]: Testing the functionality of default frame name", ros::this_node::getName().c_str());

  int result = 1;

  auto tfr = mrs_lib::Transformer("Transformer_default_frame_test");
  tfr.setDefaultPrefix("uav66");
  tfr.setLookupTimeout(ros::Duration(0.1));

  const ros::Time t = ros::Time::now();
  publish_transforms(t);

  auto tf_opt = tfr.getTransform("camera", "");
  if (tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = tfr.getTransform("", "fcu");
  if (tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "got the transform (that should not happen)");
    result *= 0;
  }

  tf_opt = std::nullopt;
  tf_opt = tfr.getTransform("", "");
  if (tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "got the transform (that should not happen)");
    result *= 0;
  }

  ROS_INFO("[%s]: Setting the default frame name", ros::this_node::getName().c_str());
  tfr.setDefaultFrame("uav66/fcu");
  tf_opt = std::nullopt;
  tf_opt = tfr.getTransform("", "");
  if (!tf_opt.has_value())
  {
    ROS_ERROR_THROTTLE(1.0, "didn't get the transformation!");
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "transformer_tests");
  ros::NodeHandle nh = ros::NodeHandle("~");
  bc = std::make_unique<tf2_ros::TransformBroadcaster>();

  local2local = anax_t(3.2, vec3_t::UnitZ());
  local2local.translation() = 10*vec3_t::Random();
  local2fcu = anax_t(1.2, vec3_t::UnitZ()) * anax_t(0.2, vec3_t::UnitX());
  local2fcu.translation() = vec3_t::Random();
  local2utm = anax_t(2.2, vec3_t::UnitZ()) * anax_t(0.1, vec3_t::UnitX());
  local2utm.translation() = vec3_t::Random();
  fcu2cam = anax_t(1.54, vec3_t::UnitZ()) * anax_t(1.54, vec3_t::UnitX()) * anax_t(1.54, vec3_t::UnitY());
  fcu2cam.translation() = vec3_t(0.2, 0, 0);

  ros::Time::waitForValid();

  std::cout << "Initialized, running tests\n";

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
