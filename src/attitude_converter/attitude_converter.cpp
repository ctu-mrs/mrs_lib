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

/* class Vector3Converter //{ */

Vector3Converter::Vector3Converter(const Eigen::Vector3d& vector3) {

  vector3_[0] = vector3[0];
  vector3_[1] = vector3[1];
  vector3_[2] = vector3[2];
}

Vector3Converter::Vector3Converter(const geometry_msgs::Vector3& vector3) {

  vector3_[0] = vector3.x;
  vector3_[1] = vector3.y;
  vector3_[2] = vector3.z;
}

Vector3Converter::Vector3Converter(const double& x, const double& y, const double& z) {

  vector3_[0] = x;
  vector3_[1] = y;
  vector3_[2] = z;
}

Vector3Converter::operator tf2::Vector3() const {

  return vector3_;
}

Vector3Converter::operator Eigen::Vector3d() const {

  return Eigen::Vector3d(vector3_[0], vector3_[1], vector3_[2]);
}

Vector3Converter::operator geometry_msgs::Vector3() const {

  geometry_msgs::Vector3 vector_3;

  vector_3.x = vector3_[0];
  vector_3.y = vector3_[1];
  vector_3.z = vector3_[2];

  return vector_3;
}

//}

/* constructors //{ */

AttitudeConverter::AttitudeConverter(const double& roll, const double& pitch, const double& yaw, const RPY_convention_t& format) {

  switch (format) {
    case RPY_EXTRINSIC: {
      tf2_quaternion_.setRPY(roll, pitch, yaw);
      break;
    }
    case RPY_INTRINSIC: {
      tf2_quaternion_.setRPY(yaw, pitch, roll);
      break;
    }
  }
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

AttitudeConverter::AttitudeConverter(const tf2::Matrix3x3 matrix) {

  matrix.getRotation(tf2_quaternion_);
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

AttitudeConverter::operator tf2::Matrix3x3() const {

  return tf2::Matrix3x3(tf2_quaternion_);
}

AttitudeConverter::operator tf2::Transform() const {

  return tf2::Transform(tf2_quaternion_);
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

double AttitudeConverter::getHeading(void) {

  tf2::Vector3 b1 = tf2::Vector3(1, 0, 0);

  tf2::Vector3 x_new = tf2::Transform(tf2_quaternion_) * b1;

  if (fabs(x_new[0]) <= 1e-3 && fabs(x_new[1]) <= 1e-3) {
    throw GetHeadingException();
  }

  return atan2(x_new[1], x_new[0]);
}

double AttitudeConverter::getYawRateIntrinsic(const double& heading_rate) {

  // prep
  Eigen::Matrix3d R = *this;

  // construct the heading orbital velocity vector
  Eigen::Vector3d heading_vector        = Eigen::Vector3d(R(0, 0), R(1, 0), 0);
  Eigen::Vector3d orbital_velocity      = Eigen::Vector3d(0, 0, heading_rate).cross(heading_vector);

  // projector to the heading orbital velocity vector subspace
  Eigen::Vector3d b_orb = Eigen::Vector3d(0, 0, 1).cross(heading_vector);
  b_orb.normalize();
  Eigen::Matrix3d P = b_orb * b_orb.transpose();

  // project the body yaw orbital velocity vector base onto the heading orbital velocity vector subspace
  Eigen::Vector3d projected = P * R.col(1);

  Eigen::Vector3d body_yaw_orbital_velocity = R.col(1) * (orbital_velocity[0] / projected[0]);

  // extract the yaw rate
  return body_yaw_orbital_velocity.norm();
}

double AttitudeConverter::getHeadingRate(const Vector3Converter& attitude_rate) {

  // prep
  Eigen::Matrix3d R = *this;
  Eigen::Vector3d w = attitude_rate;

  // create the angular velocity tensor
  Eigen::Matrix3d W;
  W << 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0;

  // R derivative
  Eigen::Matrix3d R_d = R * W;

  // atan2 derivative
  double rx        = R(0, 0);  // x-component of body X
  double ry        = R(1, 0);  // y-component of body Y
  double atan2_d_x = -ry / (rx * rx + ry * ry);
  double atan2_d_y = rx / (rx * rx + ry * ry);

  // atan2 total differential
  double heading_rate = atan2_d_x * R_d(0, 0) + atan2_d_y * R_d(1, 0);

  return heading_rate;
}

Vector3Converter AttitudeConverter::getVectorX(void) {

  tf2::Vector3 b1 = tf2::Vector3(1, 0, 0);

  tf2::Vector3 new_b1 = tf2::Transform(tf2_quaternion_) * b1;

  return Vector3Converter(new_b1);
}

Vector3Converter AttitudeConverter::getVectorY(void) {

  tf2::Vector3 b2 = tf2::Vector3(0, 1, 0);

  tf2::Vector3 new_b2 = tf2::Transform(tf2_quaternion_) * b2;

  return Vector3Converter(new_b2);
}

Vector3Converter AttitudeConverter::getVectorZ(void) {

  tf2::Vector3 b3 = tf2::Vector3(0, 0, 1);

  tf2::Vector3 new_b3 = tf2::Transform(tf2_quaternion_) * b3;

  return Vector3Converter(new_b3);
}

std::tuple<double, double, double> AttitudeConverter::getExtrinsicRPY(void) {

  auto [roll, pitch, yaw] = *this;

  return std::tuple(roll, pitch, yaw);
}

std::tuple<double, double, double> AttitudeConverter::getIntrinsicRPY(void) {

  auto [roll_e, pitch_e, yaw_e] = *this;

  return std::tuple(yaw_e, pitch_e, roll_e);
}

//}

/* setters //{ */

AttitudeConverter AttitudeConverter::setHeadingByYaw(const double& heading) {

  // get the X and Z unit vector after the original rotation
  Eigen::Vector3d b1 = getVectorX();
  Eigen::Vector3d b3 = getVectorZ();

  // check for singularity: z component of the thrust vector is 0
  if (fabs(b3[2]) < 1e-3) {
    throw SetHeadingByYawException();
  }

  // get the desired heading as a vector in 3D
  Eigen::Vector3d h(cos(heading), sin(heading), 0);

  // cast down the heading vector to the plane orthogonal to b3 (the thrust vector)
  Eigen::Vector3d heading_vec_sklop(h[0], h[1], (-b3[0] * h[0] - b3[1] * h[1]) / b3[2]);

  // get the relative angle between the projected heading and b1
  double yaw_diff = mrs_lib::vectorAngle(b1, heading_vec_sklop);

  // get the rotation around b3 about yaw_diff
  Eigen::Matrix3d rotator = Eigen::AngleAxisd(-yaw_diff, b3).toRotationMatrix();

  // get the original rotation in the matrix form
  Eigen::Matrix3d original_attitude = *this;

  // concatenate the transformations
  Eigen::Matrix3d new_attitude = rotator * original_attitude;

  return AttitudeConverter(new_attitude);
}

AttitudeConverter AttitudeConverter::setYaw(const double& new_yaw) {

  auto [roll, pitch, yaw] = *this;

  std::ignore = yaw;

  return AttitudeConverter(roll, pitch, new_yaw);
}

AttitudeConverter AttitudeConverter::setHeading(const double& heading) {

  // get the Z unit vector after the original rotation
  Eigen::Vector3d b3 = getVectorZ();

  // check for singularity: z component of the thrust vector is 0
  if (fabs(b3[2]) < 1e-3) {
    throw SetHeadingByYawException();
  }

  // get the desired heading as a vector in 3D
  Eigen::Vector3d h(cos(heading), sin(heading), 0);

  Eigen::Vector3d b2_new = b3.cross(h);
  b2_new.normalize();

  Eigen::Vector3d b1_new = b2_new.cross(b3);

  Eigen::Matrix3d new_R;
  new_R.col(0) = b1_new;
  new_R.col(1) = b2_new;
  new_R.col(2) = b3;

  return AttitudeConverter(new_R);
}

//}

// | ------------------------ internal ------------------------ |

void AttitudeConverter::calculateRPY(void) {

  if (!got_rpy_) {

    tf2::Matrix3x3(tf2_quaternion_).getRPY(roll_, pitch_, yaw_);
  }
}

}  // namespace mrs_lib
