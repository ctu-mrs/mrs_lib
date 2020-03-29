// clang: TomasFormat

#include <mrs_lib/attitude_converter.h>

using quat_t = Eigen::Quaterniond;

namespace mrs_lib
{

/* class EulerAttitude //{ */

EulerAttitude::EulerAttitude(const double& roll, const double& pitch, const double& yaw) : roll_(roll), pitch_(pitch), yaw_(yaw){};

double EulerAttitude::roll(void) const {
  return roll_;
}

double EulerAttitude::pitch(void) const {
  return pitch_;
}

double EulerAttitude::yaw(void) const {
  return yaw_;
}

//}

/* constructors //{ */

AttitudeConverter::AttitudeConverter(const double& roll, const double& pitch, const double& yaw) {
  tf2_quaternion_.setRPY(roll, pitch, yaw);
}

AttitudeConverter::AttitudeConverter(const tf::Quaternion quaternion) {
  tf2_quaternion_.setX(quaternion.x());
  tf2_quaternion_.setY(quaternion.y());
  tf2_quaternion_.setZ(quaternion.z());
  tf2_quaternion_.setW(quaternion.w());
}

AttitudeConverter::AttitudeConverter(const geometry_msgs::Quaternion quaternion) {
  tf2_quaternion_.setX(quaternion.x);
  tf2_quaternion_.setY(quaternion.y);
  tf2_quaternion_.setZ(quaternion.z);
  tf2_quaternion_.setW(quaternion.w);
}

AttitudeConverter::AttitudeConverter(const mrs_lib::EulerAttitude& euler_attitude) {
  tf2_quaternion_.setRPY(euler_attitude.roll(), euler_attitude.pitch(), euler_attitude.yaw());
}

AttitudeConverter::AttitudeConverter(const Eigen::Quaterniond quaternion) {
  tf2_quaternion_.setX(quaternion.x());
  tf2_quaternion_.setY(quaternion.y());
  tf2_quaternion_.setZ(quaternion.z());
  tf2_quaternion_.setW(quaternion.w());
}

AttitudeConverter::AttitudeConverter(const Eigen::Matrix3d matrix) {
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(matrix);

  tf2_quaternion_.setX(quaternion.x());
  tf2_quaternion_.setY(quaternion.y());
  tf2_quaternion_.setZ(quaternion.z());
  tf2_quaternion_.setW(quaternion.w());
}

AttitudeConverter::AttitudeConverter(const tf2::Quaternion quaternion) {

  tf2_quaternion_ = quaternion;
}

//}

/* operators //{ */

AttitudeConverter::operator tf2::Quaternion() const {
  return tf2_quaternion_;
}

AttitudeConverter::operator tf::Quaternion() const {

  tf::Quaternion tf_quaternion;

  tf_quaternion.setX(tf2_quaternion_.x());
  tf_quaternion.setY(tf2_quaternion_.y());
  tf_quaternion.setZ(tf2_quaternion_.z());
  tf_quaternion.setW(tf2_quaternion_.w());

  return tf_quaternion;
}

AttitudeConverter::operator geometry_msgs::Quaternion() const {

  geometry_msgs::Quaternion geom_quaternion;

  geom_quaternion.x = tf2_quaternion_.x();
  geom_quaternion.y = tf2_quaternion_.y();
  geom_quaternion.z = tf2_quaternion_.z();
  geom_quaternion.w = tf2_quaternion_.w();

  return geom_quaternion;
}

AttitudeConverter::operator EulerAttitude() const {

  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quaternion_).getRPY(roll, pitch, yaw);

  return EulerAttitude(roll, pitch, yaw);
}

AttitudeConverter::operator Eigen::Matrix3d() const {
  Eigen::Quaterniond quaternion(tf2_quaternion_.w(), tf2_quaternion_.x(), tf2_quaternion_.y(), tf2_quaternion_.z());

  return quaternion.toRotationMatrix();
}

AttitudeConverter::operator std::tuple<double&, double&, double&>() {

  tf2::Matrix3x3(tf2_quaternion_).getRPY(roll_, pitch_, yaw_);

  return std::tuple<double&, double&, double&>{roll_, pitch_, yaw_};
}

//}

/* getters //{ */

double AttitudeConverter::getYaw(void) {

  calculateRPY();

  return yaw_;
}

double AttitudeConverter::getRoll(void) {

  calculateRPY();

  return roll_;
}

double AttitudeConverter::getPitch(void) {

  calculateRPY();

  return pitch_;
}

//}

void AttitudeConverter::calculateRPY(void) {

  if (!got_rpy_) {

    tf2::Matrix3x3(tf2_quaternion_).getRPY(roll_, pitch_, yaw_);
  }
}

}  // namespace mrs_lib
