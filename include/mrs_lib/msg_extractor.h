// clang: TomasFormat
/**  \file
 *   \brief utility functions for getting stuff from ROS msgs
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef MRS_LIB_MSG_EXTRACTOR_H
#define MRS_LIB_MSG_EXTRACTOR_H

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ReferenceStamped.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/attitude_converter.h>

namespace mrs_lib
{

/* geometry_msgs::Point //{ */

/**
 * @brief get XYZ from geometry_msgs::Point
 *
 * @param data point
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::Point& data) {

  double x = data.x;
  double y = data.y;
  double z = data.z;

  return std::tuple(x, y, z);
}

/**
 * @brief get XYZ from geometry_msgs::PointConstPtr
 *
 * @param data point (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::PointConstPtr& data) {

  return getXYZ(*data);
}

//}

/* geometry_msgs::Vector3 //{ */

/**
 * @brief get XYZ from geometry_msgs::Vector3
 *
 * @param data vector3
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::Vector3& data) {

  double x = data.x;
  double y = data.y;
  double z = data.z;

  return std::tuple(x, y, z);
}

/**
 * @brief get XYZ from geometry_msgs::Vector3ConstPtr
 *
 * @param data vector3 (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::Vector3ConstPtr& data) {

  return getXYZ(*data);
}

//}

/* geometry_msgs::Pose //{ */

/* getPosition() //{ */

/**
 * @brief get position from geometry_msgs::Pose
 *
 * @param data pose
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::Pose& data) {

  return getXYZ(data.position);
}

/**
 * @brief get position from geometry_msgs::PoseConstPtr
 *
 * @param data pose (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::PoseConstPtr& data) {

  return getPosition(*data);
}

//}

/* getHeading() //{ */

/**
 * @brief get heading from geometry_msgs::Pose
 *
 * @param data pose
 *
 * @return heading
 */
double getHeading(const geometry_msgs::Pose& data) {

  return mrs_lib::AttitudeConverter(data.orientation).getHeading();
}

/**
 * @brief get heading from geometry_msgs::PoseConstPtr
 *
 * @param data pose (ConstPtr)
 *
 * @return heading
 */
double getHeading(const geometry_msgs::PoseConstPtr& data) {

  return getHeading(*data);
}

//}

/* getYaw() //{ */

/**
 * @brief get yaw from geometry_msgs::Pose
 *
 * @param data pose
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::Pose& data) {

  return mrs_lib::AttitudeConverter(data.orientation).getYaw();
}

/**
 * @brief get yaw from geometry_msgs::PoseConstPtr
 *
 * @param data pose (ConstPtr)
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::PoseConstPtr& data) {

  return getYaw(*data);
}

//}

//}

/* geometry_msgs::PoseWithCovariance //{ */

/* getPosition() //{ */

/**
 * @brief get position from geometry_msgs::PoseWithCovariance
 *
 * @param data pose with covariance
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::PoseWithCovariance& data) {

  return getPosition(data.pose);
}

/**
 * @brief get position from geometry_msgs::PoseWithCovarianceConstPtr
 *
 * @param data pose with covariance (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::PoseWithCovarianceConstPtr& data) {

  return getPosition(*data);
}

//}

/* getHeading() //{ */

/**
 * @brief get heading from geometry_msgs::PoseWithCovariance
 *
 * @param data pose with covariance
 *
 * @return heading
 */
double getHeading(const geometry_msgs::PoseWithCovariance& data) {

  return getHeading(data.pose);
}

/**
 * @brief get heading from geometry_msgs::PoseWithCovarianceConstPtr
 *
 * @param data pose with covariance (ConstPtr)
 *
 * @return heading
 */
double getHeading(const geometry_msgs::PoseWithCovarianceConstPtr& data) {

  return getHeading(*data);
}

//}

/* getYaw() //{ */

/**
 * @brief get yaw from geometry_msgs::PoseWithCovariance
 *
 * @param data pose with covariance
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::PoseWithCovariance& data) {

  return getYaw(data.pose);
}

/**
 * @brief get yaw from geometry_msgs::PoseWithCovarianceConstPtr
 *
 * @param data pose with covariance (ConstPtr)
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::PoseWithCovarianceConstPtr& data) {

  return getYaw(*data);
}

//}

//}

/* geometry_msgs::Twist //{ */

/* getVelocity() //{ */

/**
 * @brief get velocity from geometry_msgs::Twist
 *
 * @param data twist
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::Twist& data) {

  return getXYZ(data.linear);
}

/**
 * @brief get position from geometry_msgs::TwistConstPtr
 *
 * @param data twist (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::TwistConstPtr& data) {

  return getVelocity(*data);
}

//}

//}

/* geometry_msgs::TwistWithCovariance //{ */

/* getVelocity() //{ */

/**
 * @brief get velocity from geometry_msgs::TwistWithCovariance
 *
 * @param data twistwithcovariance
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::TwistWithCovariance& data) {

  return getVelocity(data.twist);
}

/**
 * @brief get position from geometry_msgs::TwistWithCovarianceConstPtr
 *
 * @param data twistwithcovariance (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::TwistWithCovarianceConstPtr& data) {

  return getVelocity(*data);
}

//}

//}

/* nav_msgs::Odometry //{ */

/* getPosition() //{ */

/**
 * @brief get position from nav_msgs::Odometry
 *
 * @param data odometry
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const nav_msgs::Odometry& data) {

  return getPosition(data.pose);
}

/**
 * @brief get position from nav_msgs::OdometryConstPtr
 *
 * @param data odometry (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const nav_msgs::OdometryConstPtr& data) {

  return getPosition(*data);
}

//}

/* getVelocity() //{ */

/**
 * @brief get position from nav_msgs::Odometry
 *
 * @param data odometry
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const nav_msgs::Odometry& data) {

  return getVelocity(data.twist);
}

/**
 * @brief get velocity from nav_msgs::OdometryConstPtr
 *
 * @param data odometry (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const nav_msgs::OdometryConstPtr& data) {

  return getVelocity(*data);
}

//}

/* getHeading() //{ */

/**
 * @brief get heading from nav_msgs::Odometry
 *
 * @param data odometry
 *
 * @return heading
 */
double getHeading(const nav_msgs::Odometry& data) {

  return getHeading(data.pose);
}

/**
 * @brief get heading from nav_msgs::OdometryConstPtr
 *
 * @param data odometry (ConstPtr)
 *
 * @return heading
 */
double getHeading(const nav_msgs::OdometryConstPtr& data) {

  return getHeading(*data);
}

//}

/* getYaw() //{ */

/**
 * @brief get yaw from nav_msgs::Odometry
 *
 * @param data odometry
 *
 * @return yaw
 */
double getYaw(const nav_msgs::Odometry& data) {

  return getYaw(data.pose);
}

/**
 * @brief get yaw from nav_msgs::OdometryConstPtr
 *
 * @param data odometry (ConstPtr)
 *
 * @return yaw
 */
double getYaw(const nav_msgs::OdometryConstPtr& data) {

  return getYaw(*data);
}

//}

/* getPose() //{ */

/**
 * @brief returns the Pose part of the nav_msgs::Odometry message
 *
 * @param data odometry
 *
 * @return pose
 */
geometry_msgs::Pose getPose(const nav_msgs::Odometry& data) {

  return data.pose.pose;
}

/**
 * @brief returns the Pose part of the nav_msgs::OdometryConstPtr message
 *
 * @param data odometry (ConstPtr)
 *
 * @return pose
 */
geometry_msgs::Pose getPose(const nav_msgs::OdometryConstPtr& data) {

  return getPose(*data);
}

//}

//}

/* mrs_msgs::PositionCommand //{ */

/* getPosition() //{ */

/**
 * @brief get position data from mrs_msgs::PositionCommand
 *
 * @param data position command
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::PositionCommand& data) {

  return getXYZ(data.position);
}

/**
 * @brief get position data from mrs_msgs::PositionCommandConstPtr
 *
 * @param data position command (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::PositionCommandConstPtr& data) {

  return getPosition(*data);
}

//}

/* getVelocity() //{ */

/**
 * @brief get velocity data from mrs_msgs::PositionCommand
 *
 * @param data position command
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const mrs_msgs::PositionCommand& data) {

  return getXYZ(data.velocity);
}

/**
 * @brief get velocity data from mrs_msgs::PositionCommandConstPtr
 *
 * @param data position command (ConstPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const mrs_msgs::PositionCommandConstPtr& data) {

  return getVelocity(*data);
}

//}

/* getHeading() //{ */

/**
 * @brief get heading from mrs_msgs::PositionCommand
 *
 * @param data position command
 *
 * @return heading
 */
double getHeading(const mrs_msgs::PositionCommand& data) {

  double heading = 0;

  if (data.use_heading) {

    heading = data.heading;

  } else if (data.use_orientation) {

    heading = mrs_lib::AttitudeConverter(data.orientation).getHeading();
  }

  return heading;
}

/**
 * @brief get heading from mrs_msgs::PositionCommandConstPtr
 *
 * @param data position command (ConstPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::PositionCommandConstPtr& data) {

  return getHeading(*data);
}

//}

//}

/* mrs_msgs::Reference //{ */

/* getPosition() //{ */

/**
 * @brief get position from mrs_msgs::Reference
 *
 * @param data reference
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::Reference& data) {

  return getXYZ(data.position);
}

/**
 * @brief get position from mrs_msgs::ReferenceConstPtr
 *
 * @param data reference (ContrPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::ReferenceConstPtr& data) {

  return getPosition(*data);
}

//}

/* getHeading() //{ */

/**
 * @brief get heading from mrs_msgs::Reference
 *
 * @param data reference
 *
 * @return heading
 */
double getHeading(const mrs_msgs::Reference& data) {

  return data.heading;
}

/**
 * @brief get heading from mrs_msgs::ReferenceConstPtr
 *
 * @param data reference (ContrPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::ReferenceConstPtr& data) {

  return getHeading(*data);
}

//}

//}

/* mrs_msgs::ReferenceStamped //{ */

/* getPosition() //{ */

/**
 * @brief get position from mrs_msgs::ReferenceStamped
 *
 * @param data reference stamped
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::ReferenceStamped& data) {

  return getPosition(data.reference);
}

/**
 * @brief get position from mrs_msgs::ReferenceStampedConstPtr
 *
 * @param data reference stamped (ContrPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::ReferenceStampedConstPtr& data) {

  return getPosition(*data);
}

//}

/* getHeading() //{ */

/**
 * @brief get heading from mrs_msgs::ReferenceStamped
 *
 * @param data referencestamped
 *
 * @return heading
 */
double getHeading(const mrs_msgs::ReferenceStamped& data) {

  return getHeading(data.reference);
}

/**
 * @brief get heading from mrs_msgs::ReferenceStampedConstPtr
 *
 * @param data referencestamped (ContrPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::ReferenceStampedConstPtr& data) {

  return getHeading(*data);
}

//}

//}

}  // namespace mrs_lib

//}

#endif  // MRS_LIB_MSG_EXTRACTOR_H
