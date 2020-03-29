// clang: TomasFormat
/**
 * @file attitude_converter.h
 *
 * @brief Conversions between various representations of object attitude in 3D.
 * Supports Quaternions, Euler angles, Angle-axis and Rotational matrices from tf, tf2, Eigen and geometry_msgs libraries.
 * The default Euler angle notation is the extrinsic RPY.
 *
 * @author Tomas Baca
 * @version 1.0
 * @date 2020-03-29
 */

#ifndef ATTITUDE_CONVERTER_H
#define ATTITUDE_CONVERTER_H

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <tuple>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

namespace mrs_lib
{

/**
 * @brief A small class for storing the Euler angles.
 */
class EulerAttitude {
public:
  /**
   * @brief A simple class for storing the Euler angles.
   *
   * @param roll
   * @param pitch
   * @param yaw
   */
  EulerAttitude(const double& roll, const double& pitch, const double& yaw);

  /**
   * @brief get the roll angle
   *
   * @return roll
   */
  double roll(void) const;

  /**
   * @brief get the pitch angle
   *
   * @return pitch
   */
  double pitch(void) const;

  /**
   * @brief get the yaw angle
   *
   * @return yaw
   */
  double yaw(void) const;

private:
  double roll_, pitch_, yaw_;
};

/**
 * @brief The main convertor class. Instantiate with any type in constructor and get the value in any other type by assigning the instance to your variable, as:
 *   tf::Quaternion tf1_quaternion = AttitudeConverter(roll, pitch, yaw); All the default Euler angles are in the extrinsic RPY notation.
 */
class AttitudeConverter {
public:
  /* constructors //{ */


  /**
   * @brief Euler angles constructor
   *
   * @param roll
   * @param pitch
   * @param yaw
   */
  AttitudeConverter(const double& roll, const double& pitch, const double& yaw);

  /**
   * @brief tf::Quaternion constructor
   *
   * @param quaternion tf::Quaternion quaternion
   */
  AttitudeConverter(const tf::Quaternion quaternion);

  /**
   * @brief geometry_msgs::Quaternion constructor
   *
   * @param quaternion geometry_msgs::Quaternion quaternion
   */
  AttitudeConverter(const geometry_msgs::Quaternion quaternion);

  /**
   * @brief mrs_lib::EulerAttitude constructor
   *
   * @param euler_attitude mrs_lib::EulerAttitude
   */
  AttitudeConverter(const mrs_lib::EulerAttitude& euler_attitude);

  /**
   * @brief Eigen::Quaterniond constructor
   *
   * @param quaternion Eigen::Quaterniond quaternion
   */
  AttitudeConverter(const Eigen::Quaterniond quaternion);

  /**
   * @brief Eigen::Matrix3d constructor
   *
   * @param matrix Eigen::Matrix3d rotational matrix
   */
  AttitudeConverter(const Eigen::Matrix3d matrix);

  /**
   * @brief Eigen::AngleAxis constructor
   *
   * @tparam T angle-axis base type
   * @param angle_axis Eigen::AngleAxis
   */
  template <class T>
  AttitudeConverter(const Eigen::AngleAxis<T> angle_axis) {
    double       angle = angle_axis.angle();
    tf2::Vector3 axis(angle_axis.axis()[0], angle_axis.axis()[1], angle_axis.axis()[2]);

    tf2_quaternion_.setRotation(axis, angle);
  }

  /**
   * @brief tf2::Quaternion constructor
   *
   * @param quaternion tf2::Quaternion
   */
  AttitudeConverter(const tf2::Quaternion quaternion);

  //}

  /* operators //{ */

  /**
   * @brief typecast to tf2::Quaternion
   *
   * @return orientation in tf2::Quaternion
   */
  operator tf2::Quaternion() const;

  /**
   * @brief typecast to tf::Quaternion
   *
   * @return orientation in tf::Quaternion
   */
  operator tf::Quaternion() const;

  /**
   * @brief typecast to geometry_msgs::Quaternion
   *
   * @return orientation in geometry_msgs::Quaternion
   */
  operator geometry_msgs::Quaternion() const;

  /**
   * @brief typecast to EulerAttitude
   *
   * @return orientation in EulerAttitude
   */
  operator EulerAttitude() const;

  /**
   * @brief typecast to Eigen::AngleAxis
   *
   * @tparam T angle-axis base type
   *
   * @return orientation in EulerAttitude
   */
  template <class T>
  operator Eigen::AngleAxis<T>() const {

    double          angle = tf2_quaternion_.getAngle();
    Eigen::Vector3d axis(tf2_quaternion_.getAxis()[0], tf2_quaternion_.getAxis()[1], tf2_quaternion_.getAxis()[2]);

    Eigen::AngleAxis<T> angle_axis(angle, axis);

    return angle_axis;
  }


  /**
   * @brief typecast to EulerAttitude Eigen::Quaternion
   *
   * @tparam T quaternion base type
   *
   * @return orientation in Eigen::Quaternion
   */
  template <class T>
  operator Eigen::Quaternion<T>() const {

    return Eigen::Quaternion<T>(tf2_quaternion_.w(), tf2_quaternion_.x(), tf2_quaternion_.y(), tf2_quaternion_.z());
  }

  operator Eigen::Matrix3d() const;

  /**
   * @brief typecase to tuple of Euler angles in extrinsic RPY
   *
   * @return std::tuple of extrinsic RPY
   */
  operator std::tuple<double&, double&, double&>();

  //}

  /* getters //{ */

  /**
   * @brief get the roll angle
   *
   * @return roll
   */
  double getRoll(void);

  /**
   * @brief get the pitch angle
   *
   * @return pitch
   */
  double getPitch(void);

  /**
   * @brief get the yaw angle
   *
   * @return yaw
   */
  double getYaw(void);

  //}

  template <std::size_t I>
  constexpr auto get();

private:
  /**
   * @brief Internal representation of the attitude
   */
  tf2::Quaternion tf2_quaternion_;

  /**
   * @brief convert the internal quaternion representation to internally-stored RPY
   */
  void calculateRPY(void);

  /**
   * @brief Internal representation in RPY. is used only when converting to RPY.
   */
  double roll_, pitch_, yaw_;
  bool   got_rpy_ = false;
};


template <std::size_t I>
constexpr auto AttitudeConverter::get() {

  calculateRPY();

  // call compilation error if I > 2
  static_assert(I <= 2);

  // get the RPY components based on the index in the tuple
  if constexpr (I == 0) {
    return static_cast<double>(roll_);
  } else if constexpr (I == 1) {
    return static_cast<double>(pitch_);
  } else if constexpr (I == 2) {
    return static_cast<double>(yaw_);
  }
}

}  // namespace mrs_lib

template <>
struct std::tuple_size<mrs_lib::AttitudeConverter>
{ static constexpr int value = 3; };

template <>
struct std::tuple_element<0, mrs_lib::AttitudeConverter>
{ using type = double; };

template <>
struct std::tuple_element<1, mrs_lib::AttitudeConverter>
{ using type = double; };

template <>
struct std::tuple_element<2, mrs_lib::AttitudeConverter>
{ using type = double; };

#endif
