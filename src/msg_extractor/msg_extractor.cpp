#include <mrs_lib/msg_extractor.h>

namespace mrs_lib
{

/* geometry_msgs::msg::Point //{ */

std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Point& data) {

  double x = data.x;
  double y = data.y;
  double z = data.z;

  return std::tuple(x, y, z);
}

std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Point::SharedPtr& data) {

  return getXYZ(*data);
}

std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Point::ConstSharedPtr& data) {

  return getXYZ(*data);
}

//}

/* geometry_msgs::msg::Vector3 //{ */

std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Vector3& data) {

  double x = data.x;
  double y = data.y;
  double z = data.z;

  return std::tuple(x, y, z);
}

std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Vector3::SharedPtr& data) {

  return getXYZ(*data);
}

std::tuple<double, double, double> getXYZ(const geometry_msgs::msg::Vector3::ConstSharedPtr& data) {

  return getXYZ(*data);
}

//}

/* geometry_msgs::msg::Pose //{ */

std::tuple<double, double, double> getPosition(const geometry_msgs::msg::Pose& data) {

  return getXYZ(data.position);
}

std::tuple<double, double, double> getPosition(const geometry_msgs::msg::Pose::SharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getPosition(const geometry_msgs::msg::Pose::ConstSharedPtr& data) {

  return getPosition(*data);
}

double getHeading(const geometry_msgs::msg::Pose& data) {

  return mrs_lib::AttitudeConverter(data.orientation).getHeading();
}

double getHeading(const geometry_msgs::msg::Pose::SharedPtr& data) {

  return getHeading(*data);
}

double getHeading(const geometry_msgs::msg::Pose::ConstSharedPtr& data) {

  return getHeading(*data);
}

double getYaw(const geometry_msgs::msg::Pose& data) {

  return mrs_lib::AttitudeConverter(data.orientation).getYaw();
}

double getYaw(const geometry_msgs::msg::Pose::SharedPtr& data) {

  return getYaw(*data);
}

double getYaw(const geometry_msgs::msg::Pose::ConstSharedPtr& data) {

  return getYaw(*data);
}

//}

/* geometry_msgs::msg::PoseWithCovariance //{ */

std::tuple<double, double, double> getPosition(const geometry_msgs::msg::PoseWithCovariance& data) {

  return getPosition(data.pose);
}

std::tuple<double, double, double> getPosition(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getPosition(const geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr& data) {

  return getPosition(*data);
}

double getHeading(const geometry_msgs::msg::PoseWithCovariance& data) {

  return getHeading(data.pose);
}

double getHeading(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& data) {

  return getHeading(*data);
}

double getHeading(const geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr& data) {

  return getHeading(*data);
}

double getYaw(const geometry_msgs::msg::PoseWithCovariance& data) {

  return getYaw(data.pose);
}

double getYaw(const geometry_msgs::msg::PoseWithCovariance::SharedPtr& data) {

  return getYaw(*data);
}

double getYaw(const geometry_msgs::msg::PoseWithCovariance::ConstSharedPtr& data) {

  return getYaw(*data);
}

//}

/* geometry_msgs::msg::Twist //{ */

std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::Twist& data) {

  return getXYZ(data.linear);
}

std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::Twist::SharedPtr& data) {

  return getVelocity(*data);
}

std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::Twist::ConstSharedPtr& data) {

  return getVelocity(*data);
}

//}

/* geometry_msgs::msg::TwistWithCovariance //{ */

std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::TwistWithCovariance& data) {

  return getVelocity(data.twist);
}

std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::TwistWithCovariance::SharedPtr& data) {

  return getVelocity(*data);
}

std::tuple<double, double, double> getVelocity(const geometry_msgs::msg::TwistWithCovariance::ConstSharedPtr& data) {

  return getVelocity(*data);
}

//}

/* nav_msgs::msg::Odometry //{ */

std::tuple<double, double, double> getPosition(const nav_msgs::msg::Odometry& data) {

  return getPosition(data.pose);
}

std::tuple<double, double, double> getPosition(const nav_msgs::msg::Odometry::SharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getPosition(const nav_msgs::msg::Odometry::ConstSharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getVelocity(const nav_msgs::msg::Odometry& data) {

  return getVelocity(data.twist);
}

std::tuple<double, double, double> getVelocity(const nav_msgs::msg::Odometry::SharedPtr& data) {

  return getVelocity(*data);
}

std::tuple<double, double, double> getVelocity(const nav_msgs::msg::Odometry::ConstSharedPtr& data) {

  return getVelocity(*data);
}

double getHeading(const nav_msgs::msg::Odometry& data) {

  return getHeading(data.pose);
}

double getHeading(const nav_msgs::msg::Odometry::SharedPtr& data) {

  return getHeading(*data);
}

double getHeading(const nav_msgs::msg::Odometry::ConstSharedPtr& data) {

  return getHeading(*data);
}

double getYaw(const nav_msgs::msg::Odometry& data) {

  return getYaw(data.pose);
}

double getYaw(const nav_msgs::msg::Odometry::SharedPtr& data) {

  return getYaw(*data);
}

double getYaw(const nav_msgs::msg::Odometry::ConstSharedPtr& data) {

  return getYaw(*data);
}

geometry_msgs::msg::Pose getPose(const nav_msgs::msg::Odometry& data) {

  return data.pose.pose;
}

geometry_msgs::msg::Pose getPose(const nav_msgs::msg::Odometry::SharedPtr& data) {

  return getPose(*data);
}

geometry_msgs::msg::Pose getPose(const nav_msgs::msg::Odometry::ConstSharedPtr& data) {

  return getPose(*data);
}

//}

/* mrs_msgs::msg::TrackerCommand //{ */

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::TrackerCommand& data) {

  return getXYZ(data.position);
}

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::TrackerCommand::SharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getVelocity(const mrs_msgs::msg::TrackerCommand& data) {

  return getXYZ(data.velocity);
}

std::tuple<double, double, double> getVelocity(const mrs_msgs::msg::TrackerCommand::SharedPtr& data) {

  return getVelocity(*data);
}

std::tuple<double, double, double> getVelocity(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr& data) {

  return getVelocity(*data);
}

double getHeading(const mrs_msgs::msg::TrackerCommand& data) {

  double heading = 0;

  if (data.use_heading) {

    heading = data.heading;

  } else if (data.use_orientation) {

    heading = mrs_lib::AttitudeConverter(data.orientation).getHeading();
  }

  return heading;
}

double getHeading(const mrs_msgs::msg::TrackerCommand::SharedPtr& data) {

  return getHeading(*data);
}

double getHeading(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr& data) {

  return getHeading(*data);
}

//}

/* mrs_msgs::msg::Reference //{ */

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::Reference& data) {

  return getXYZ(data.position);
}

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::Reference::SharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::Reference::ConstSharedPtr& data) {

  return getPosition(*data);
}

double getHeading(const mrs_msgs::msg::Reference& data) {

  return data.heading;
}

double getHeading(const mrs_msgs::msg::Reference::SharedPtr& data) {

  return getHeading(*data);
}

double getHeading(const mrs_msgs::msg::Reference::ConstSharedPtr& data) {

  return getHeading(*data);
}

//}

/* mrs_msgs::msg::ReferenceStamped //{ */

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::ReferenceStamped& data) {

  return getPosition(data.reference);
}

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::ReferenceStamped::SharedPtr& data) {

  return getPosition(*data);
}

std::tuple<double, double, double> getPosition(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr& data) {

  return getPosition(*data);
}

double getHeading(const mrs_msgs::msg::ReferenceStamped& data) {

  return getHeading(data.reference);
}

double getHeading(const mrs_msgs::msg::ReferenceStamped::SharedPtr& data) {

  return getHeading(*data);
}

double getHeading(const mrs_msgs::msg::ReferenceStamped::ConstSharedPtr& data) {

  return getHeading(*data);
}

//}

}  // namespace mrs_lib
