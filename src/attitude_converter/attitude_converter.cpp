// clang: TomasFormat

#include <cstdio>
#include <iostream>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/utils.h>

using quat_t = Eigen::Quaterniond;

namespace mrs_lib
{

/* class EulerAttitude //{ */

EulerAttitude::EulerAttitude(const double& roll, const double& pitch, const double& yaw) : roll_(roll), pitch_(pitch), yaw_(yaw) {
}

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

Vector3Converter::Vector3Converter(const geometry_msgs::msg::Vector3& vector3) {

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

Vector3Converter::operator geometry_msgs::msg::Vector3() const {

  geometry_msgs::msg::Vector3 vector_3;

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

      Eigen::Matrix3d Y, P, R;

      Y << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

      P << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

      R << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

      tf2_quaternion_ = AttitudeConverter(R * P * Y);

      break;
    }
  }

  validateOrientation();
}

AttitudeConverter::AttitudeConverter(const geometry_msgs::msg::Quaternion quaternion) {
  tf2_quaternion_.setX(quaternion.x);
  tf2_quaternion_.setY(quaternion.y);
  tf2_quaternion_.setZ(quaternion.z);
  tf2_quaternion_.setW(quaternion.w);

  validateOrientation();
}

AttitudeConverter::AttitudeConverter(const mrs_lib::EulerAttitude& euler_attitude) {
  tf2_quaternion_.setRPY(euler_attitude.roll(), euler_attitude.pitch(), euler_attitude.yaw());

  validateOrientation();
}

AttitudeConverter::AttitudeConverter(const Eigen::Quaterniond quaternion) {
  tf2_quaternion_.setX(quaternion.x());
  tf2_quaternion_.setY(quaternion.y());
  tf2_quaternion_.setZ(quaternion.z());
  tf2_quaternion_.setW(quaternion.w());

  validateOrientation();
}

AttitudeConverter::AttitudeConverter(const Eigen::Matrix3d matrix) {
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(matrix);

  tf2_quaternion_.setX(quaternion.x());
  tf2_quaternion_.setY(quaternion.y());
  tf2_quaternion_.setZ(quaternion.z());
  tf2_quaternion_.setW(quaternion.w());

  validateOrientation();
}

AttitudeConverter::AttitudeConverter(const tf2::Quaternion quaternion) {

  tf2_quaternion_ = quaternion;

  validateOrientation();
}

AttitudeConverter::AttitudeConverter(const tf2::Matrix3x3 matrix) {

  matrix.getRotation(tf2_quaternion_);

  validateOrientation();
}

//}

/* operators //{ */

AttitudeConverter::operator tf2::Quaternion() const {
  return tf2_quaternion_;
}

AttitudeConverter::operator geometry_msgs::msg::Quaternion() const {

  geometry_msgs::msg::Quaternion geom_quaternion;

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

  // when the heading rate is very small, it does not make sense to compute the
  // yaw rate (the math would break), return 0
  if (fabs(heading_rate) < 1e-3) {
    return 0;
  }

  // prep
  Eigen::Matrix3d R = *this;

  // construct the heading orbital velocity vector
  Eigen::Vector3d heading_vector   = Eigen::Vector3d(R(0, 0), R(1, 0), 0);
  Eigen::Vector3d orbital_velocity = Eigen::Vector3d(0, 0, heading_rate).cross(heading_vector);

  // projector to the heading orbital velocity vector subspace
  Eigen::Vector3d b_orb = Eigen::Vector3d(0, 0, 1).cross(heading_vector);
  b_orb.normalize();
  Eigen::Matrix3d P = b_orb * b_orb.transpose();

  // project the body yaw orbital velocity vector base onto the heading orbital velocity vector subspace
  Eigen::Vector3d projected = P * R.col(1);


  double orbital_velocity_norm = orbital_velocity.norm();
  double projected_norm        = projected.norm();

  if (fabs(projected_norm) < 1e-5) {
    std::cout << ("[AttitudeConverter]: getYawRateIntrinsic(): \"projected_norm\" in denominator is almost zero!!!") << std::endl;
    throw MathErrorException();
  }

  double direction = mrs_lib::signum(orbital_velocity.dot(projected));

  double output_yaw_rate = direction * (orbital_velocity_norm / projected_norm);

  if (!std::isfinite(output_yaw_rate)) {
    std::cout << "[AttitudeConverter]: getYawRateIntrinsic(): NaN detected in variable \"output_yaw_rate\"!!!" << std::endl;
    throw MathErrorException();
  }

  // extract the yaw rate
  return output_yaw_rate;
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
  double rx = R(0, 0);  // x-component of body X
  double ry = R(1, 0);  // y-component of body Y

  double denom = rx * rx + ry * ry;

  if (fabs(denom) <= 1e-5) {
    std::cout << "[AttitudeConverter]: getHeadingRate(): denominator near zero!!!" << std::endl;
    throw MathErrorException();
  }

  double atan2_d_x = -ry / denom;
  double atan2_d_y = rx / denom;

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

  Eigen::Matrix3d rot = AttitudeConverter(*this);

  Eigen::Vector3d eulers = rot.eulerAngles(2, 1, 0);

  return std::tuple(eulers[2], eulers[1], eulers[0]);
}

std::tuple<double, double, double> AttitudeConverter::getIntrinsicRPY(void) {

  Eigen::Matrix3d rot = AttitudeConverter(*this);

  Eigen::Vector3d eulers = rot.eulerAngles(0, 1, 2);

  return std::tuple(eulers[0], eulers[1], eulers[2]);
}

//}

/* setters //{ */

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
    throw SetHeadingException();
  }

  // get the desired heading as a vector in 3D
  Eigen::Vector3d h(cos(heading), sin(heading), 0);

  Eigen::Matrix3d new_R;

  new_R.col(2) = b3;

  // construct the oblique projection
  Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - b3 * b3.transpose());

  // create a basis of the body-z complement subspace
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
  A.col(0)          = projector_body_z_compl.col(0);
  A.col(1)          = projector_body_z_compl.col(1);

  // create the basis of the projection null-space complement
  Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
  B.col(0)          = Eigen::Vector3d(1, 0, 0);
  B.col(1)          = Eigen::Vector3d(0, 1, 0);

  // oblique projector to <range_basis>
  Eigen::MatrixXd Bt_A               = B.transpose() * A;
  Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
  Eigen::MatrixXd oblique_projector  = A * Bt_A_pseudoinverse * B.transpose();

  new_R.col(0) = oblique_projector * h;
  new_R.col(0).normalize();

  // | ------------------------- body y ------------------------- |

  new_R.col(1) = new_R.col(2).cross(new_R.col(0));
  new_R.col(1).normalize();

  return AttitudeConverter(new_R);
}

//}

// | ------------------------ internal ------------------------ |

/* calculateRPY() //{ */

void AttitudeConverter::calculateRPY(void) {

  if (!got_rpy_) {

    tf2::Matrix3x3(tf2_quaternion_).getRPY(roll_, pitch_, yaw_);
  }
}

//}

/* validateOrientation() //{ */

void AttitudeConverter::validateOrientation(void) {

  if (!std::isfinite(tf2_quaternion_.x()) || !std::isfinite(tf2_quaternion_.y()) || !std::isfinite(tf2_quaternion_.z()) ||
      !std::isfinite(tf2_quaternion_.w())) {
    throw InvalidAttitudeException();
  }
}

//}

}  // namespace mrs_lib
