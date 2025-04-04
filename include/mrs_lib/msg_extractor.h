// clang: TomasFormat
/**  \file
 *   \brief utility functions for getting stuff from ROS msgs
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef MRS_LIB_MSG_EXTRACTOR_H
#define MRS_LIB_MSG_EXTRACTOR_H

#include <mrs_msgs/msg/reference.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <mrs_lib/attitude_converter.h>

namespace mrs_lib
{

/* geometry_msgs::msg::Point //{ */

/**
 * @brief get XYZ from geometry_msgs::msg::Point
 *
 * @param data point
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Point& data);

/**
 * @brief get XYZ from geometry_msgs::msg::Point
 *
 * @param data point (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Point::SharedPtr& data);

/**
 * @brief get XYZ from geometry_msgs::msg::Point
 *
 * @param data point (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Point::ConstSharedPtr& data);

//}

/* geometry_msgs::msg::Vector3 //{ */

/**
 * @brief get XYZ from geometry_msgs::msg::Vector3
 *
 * @param data vector3
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Vector3& data);

/**
 * @brief get XYZ from geometry_msgs::msg::Vector3::SharedPtr
 *
 * @param data vector3 (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Vector3::SharedPtr& data);

/**
 * @brief get XYZ from geometry_msgs::msg::Vector3::ConstSharedPtr
 *
 * @param data vector3 (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Vector3::ConstSharedPtr& data);

//}

/* geometry_msgs::msg::Pose //{ */

/* getPosition() //{ */

/**
 * @brief get position from geometry_msgs::msg::Pose
 *
 * @param data pose
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::msg::Pose& data);

/**
 * @brief get position from geometry_msgs::msg::Pose::SharedPtr
 *
 * @param data pose (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::msg::Pose::SharedPtr& data);

/**
 * @brief get position from geometry_msgs::msg::Pose::ConstSharedPtr
 *
 * @param data pose (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::msg::Pose::ConstSharedPtr& data);

//}

/* getHeading() //{ */

/**
 * @brief get heading from geometry_msgs::msg::Pose
 *
 * @param data pose
 *
 * @return heading
 */
double getHeading(const geometry_msgs::msg::Pose& data);

/**
 * @brief get heading from geometry_msgs::msg::Pose::SharedPtr
 *
 * @param data pose (SharedPtr)
 *
 * @return heading
 */
double getHeading(const geometry_msgs::msg::Pose::SharedPtr& data);

/**
 * @brief get heading from geometry_msgs::msg::Pose::ConstSharedPtr
 *
 * @param data pose (ConstSharedPtr)
 *
 * @return heading
 */
double getHeading(const geometry_msgs::msg::Pose::ConstSharedPtr& data);

//}

/* getYaw() //{ */

/**
 * @brief get yaw from geometry_msgs::msg::Pose
 *
 * @param data pose
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::msg::Pose& data);

/**
 * @brief get yaw from geometry_msgs::msg::Pose::SharedPtr
 *
 * @param data pose (SharedPtr)
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::msg::Pose::SharedPtr& data);

/**
 * @brief get yaw from geometry_msgs::msg::Pose::ConstSharedPtr
 *
 * @param data pose (ConstSharedPtr)
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::msg::Pose::ConstSharedPtr& data);

//}

//}

/* geometry_msgs::msg::PoseWithCovariance //{ */

/* getPosition() //{ */

/**
 * @brief get position from geometry_msgs::msg::PoseWithCovariance
 *
 * @param data pose with covariance
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::msg::PoseWithCovariance& data);

/**
 * @brief get position from geometry_msgs::msg::PoseWithCovariance::SharedPtr
 *
 * @param data pose with covariance (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& data);

/**
 * @brief get position from geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr
 *
 * @param data pose with covariance (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr& data);

//}

/* getHeading() //{ */

/**
 * @brief get heading from geometry_msgs::msg::PoseWithCovariance
 *
 * @param data pose with covariance
 *
 * @return heading
 */
double getHeading(const geometry_msgs::msg::PoseWithCovariance& data);

/**
 * @brief get heading from geometry_msgs::msg::PoseWithCovariance::SharedPtr
 *
 * @param data pose with covariance (SharedPtr)
 *
 * @return heading
 */
double getHeading(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& data);

/**
 * @brief get heading from geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr
 *
 * @param data pose with covariance (ConstSharedPtr)
 *
 * @return heading
 */
double getHeading(const geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr& data);

//}

/* getYaw() //{ */

/**
 * @brief get yaw from geometry_msgs::msg::PoseWithCovariance
 *
 * @param data pose with covariance
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::msg::PoseWithCovariance& data);

/**
 * @brief get yaw from geometry_msgs::msg::PoseWithCovariance::SharedPtr
 *
 * @param data pose with covariance (SharedPtr)
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& data);

/**
 * @brief get yaw from geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr
 *
 * @param data pose with covariance (ConstSharedPtr)
 *
 * @return yaw
 */
double getYaw(const geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr& data);

//}

//}

/* geometry_msgs::msg::Twist //{ */

/* getVelocity() //{ */

/**
 * @brief get velocity from geometry_msgs::msg::Twist
 *
 * @param data twist
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::Twist& data);

/**
 * @brief get position from geometry_msgs::msg::Twist::SharedPtr
 *
 * @param data twist (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::Twist::SharedPtr& data);

/**
 * @brief get position from geometry_msgs::msg::Twist::ConstSharedPtr
 *
 * @param data twist (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::Twist::ConstSharedPtr& data);

//}

//}

/* geometry_msgs::msg::TwistWithCovariance //{ */

/* getVelocity() //{ */

/**
 * @brief get velocity from geometry_msgs::msg::TwistWithCovariance
 *
 * @param data twistwithcovariance
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::TwistWithCovariance& data);

/**
 * @brief get position from geometry_msgs::msg::TwistWithCovariance::SharedPtr
 *
 * @param data twistwithcovariance (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::TwistWithCovariance::SharedPtr& data);

/**
 * @brief get position from geometry_msgs::msg::TwistWithCovariance::ConstSharedPtr
 *
 * @param data twistwithcovariance (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::TwistWithCovariance::ConstSharedPtr& data);

//}

//}

/* nav_msgs::msg::Odometry //{ */

/* getPosition() //{ */

/**
 * @brief get position from nav_msgs::msg::Odometry
 *
 * @param data odometry
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const nav_msgs::msg::Odometry& data);

/**
 * @brief get position from nav_msgs::msg::Odometry::SharedPtr
 *
 * @param data odometry (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const nav_msgs::msg::Odometry::SharedPtr& data);

/**
 * @brief get position from nav_msgs::msg::Odometry::ConstSharedPtr
 *
 * @param data odometry (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const nav_msgs::msg::Odometry::ConstSharedPtr& data);

//}

/* getVelocity() //{ */

/**
 * @brief get position from nav_msgs::msg::Odometry
 *
 * @param data odometry
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const nav_msgs::msg::Odometry& data);

/**
 * @brief get velocity from nav_msgs::msg::Odometry::SharedPtr
 *
 * @param data odometry (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const nav_msgs::msg::Odometry::SharedPtr& data);

/**
 * @brief get velocity from nav_msgs::msg::Odometry::ConstSharedPtr
 *
 * @param data odometry (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const nav_msgs::msg::Odometry::ConstSharedPtr& data);


//}

/* getHeading() //{ */

/**
 * @brief get heading from nav_msgs::msg::Odometry
 *
 * @param data odometry
 *
 * @return heading
 */
double getHeading(const nav_msgs::msg::Odometry& data);

/**
 * @brief get heading from nav_msgs::msg::Odometry::SharedPtr
 *
 * @param data odometry (SharedPtr)
 *
 * @return heading
 */
double getHeading(const nav_msgs::msg::Odometry::SharedPtr& data);

/**
 * @brief get heading from nav_msgs::msg::Odometry::ConstSharedPtr
 *
 * @param data odometry (ConstSharedPtr)
 *
 * @return heading
 */
double getHeading(const nav_msgs::msg::Odometry::ConstSharedPtr& data);

//}

/* getHeading() //{ */

/**
 * @brief get heading from nav_msgs::msg::Odometry
 *
 * @param data odometry
 *
 * @return heading
 */
double getHeading(const nav_msgs::msg::Odometry& data);

/**
 * @brief get heading from nav_msgs::msg::Odometry::SharedPtr
 *
 * @param data odometry (SharedPtr)
 *
 * @return heading
 */
double getHeading(const nav_msgs::msg::Odometry::SharedPtr& data);

/**
 * @brief get heading from nav_msgs::msg::Odometry::ConstSharedPtr
 *
 * @param data odometry (ConstSharedPtr)
 *
 * @return heading
 */
double getHeading(const nav_msgs::msg::Odometry::ConstSharedPtr& data);

//}

/* getYaw() //{ */

/**
 * @brief get yaw from nav_msgs::msg::Odometry
 *
 * @param data odometry
 *
 * @return yaw
 */
double getYaw(const nav_msgs::msg::Odometry& data);

/**
 * @brief get yaw from nav_msgs::msg::Odometry::SharedPtr
 *
 * @param data odometry (SharedPtr)
 *
 * @return yaw
 */
double getYaw(const nav_msgs::msg::Odometry::SharedPtr& data);

/**
 * @brief get yaw from nav_msgs::msg::Odometry::ConstSharedPtr
 *
 * @param data odometry (ConstSharedPtr)
 *
 * @return yaw
 */
double getYaw(const nav_msgs::msg::Odometry::ConstSharedPtr& data);

//}

/* getPose() //{ */

/**
 * @brief returns the Pose part of the nav_msgs::msg::Odometry message
 *
 * @param data odometry
 *
 * @return pose
 */
geometry_msgs::msg::Pose getPose(const nav_msgs::msg::Odometry& data);

/**
 * @brief returns the Pose part of the nav_msgs::msg::Odometry::SharedPtr message
 *
 * @param data odometry (SharedPtr)
 *
 * @return pose
 */
geometry_msgs::msg::Pose getPose(const nav_msgs::msg::Odometry::SharedPtr& data);

/**
 * @brief returns the Pose part of the nav_msgs::msg::Odometry::ConstSharedPtr message
 *
 * @param data odometry (ConstSharedPtr)
 *
 * @return pose
 */
geometry_msgs::msg::Pose getPose(const nav_msgs::msg::Odometry::ConstSharedPtr& data);

//}

//}

/* mrs_msgs::msg::TrackerCommand //{ */

/* getPosition() //{ */

/**
 * @brief get position data from mrs_msgs::msg::TrackerCommand
 *
 * @param data position command
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::TrackerCommand& data);

/**
 * @brief get position data from mrs_msgs::msg::TrackerCommandSharedPtr
 *
 * @param data position command (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::TrackerCommand::SharedPtr& data);

/**
 * @brief get position data from mrs_msgs::msg::TrackerCommand::SharedPtr
 *
 * @param data position command (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr& data);


//}

/* getVelocity() //{ */

/**
 * @brief get velocity data from mrs_msgs::msg::TrackerCommand
 *
 * @param data position command
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const mrs_msgs::msg::TrackerCommand& data);

/**
 * @brief get velocity data from mrs_msgs::msg::TrackerCommand::SharedPtr
 *
 * @param data position command (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const mrs_msgs::msg::TrackerCommand::SharedPtr& data);

/**
 * @brief get velocity data from mrs_msgs::msg::TrackerCommand::ConstSharedPtr
 *
 * @param data position command (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getVelocity(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr& data);


//}

/* getHeading() //{ */

/**
 * @brief get heading from mrs_msgs::msg::TrackerCommand
 *
 * @param data position command
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::TrackerCommand& data);

/**
 * @brief get heading from mrs_msgs::msg::TrackerCommand::SharedPtr
 *
 * @param data position command (SharedPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::TrackerCommand::SharedPtr& data);

/**
 * @brief get heading from mrs_msgs::msg::TrackerCommand::ConstSharedPtr
 *
 * @param data position command (ConstSharedPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr& data);

//}

//}

/* mrs_msgs::msg::Reference //{ */

/* getPosition() //{ */

/**
 * @brief get position from mrs_msgs::msg::Reference
 *
 * @param data reference
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::Reference& data);

/**
 * @brief get position from mrs_msgs::msg::Reference::SharedPtr
 *
 * @param data reference (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::Reference::SharedPtr& data);

/**
 * @brief get position from mrs_msgs::msg::Reference::ConstSharedPtr
 *
 * @param data reference (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::Reference::ConstSharedPtr& data);

//}

/* getHeading() //{ */

/**
 * @brief get heading from mrs_msgs::msg::Reference
 *
 * @param data reference
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::Reference& data);

/**
 * @brief get heading from mrs_msgs::msg::ReferenceSharedPtr
 *
 * @param data reference (SharedPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::Reference::SharedPtr& data);

/**
 * @brief get heading from mrs_msgs::msg::ReferenceConstSharedPtr
 *
 * @param data reference (ConstSharedPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::Reference::ConstSharedPtr& data);

//}

//}

/* mrs_msgs::msg::ReferenceStamped //{ */

/* getPosition() //{ */

/**
 * @brief get position from mrs_msgs::msg::ReferenceStamped
 *
 * @param data reference stamped
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::ReferenceStamped& data);

/**
 * @brief get position from mrs_msgs::msg::ReferenceStamped::SharedPtr
 *
 * @param data reference stamped (SharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::ReferenceStamped::SharedPtr& data);

/**
 * @brief get position from mrs_msgs::msg::ReferenceStamped::ConstSharedPtr
 *
 * @param data reference stamped (ConstSharedPtr)
 *
 * @return x, y, z
 */
std::tuple<double, double, double> getPosition(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr& data);

//}

/* getHeading() //{ */

/**
 * @brief get heading from mrs_msgs::msg::ReferenceStamped
 *
 * @param data referencestamped
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::ReferenceStamped& data);

/**
 * @brief get heading from mrs_msgs::msg::ReferenceStamped::SharedPtr
 *
 * @param data referencestamped (SharedPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::ReferenceStamped::SharedPtr& data);

/**
 * @brief get heading from mrs_msgs::msg::ReferenceStamped::ConstSharedPtr
 *
 * @param data referencestamped (ConstSharedPtr)
 *
 * @return heading
 */
double getHeading(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr& data);

//}

//}

}  // namespace mrs_lib

//}

#endif  // MRS_LIB_MSG_EXTRACTOR_H
