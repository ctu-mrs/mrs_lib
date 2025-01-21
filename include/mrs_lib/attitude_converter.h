// clang: TomasFormat
/**
 * @file attitude_converter.h
 *
 * @brief Conversions between various representations of object attitude in 3D.
 * Supports Quaternions, Euler angles, Angle-axis and Rotational matrices from tf, tf2, Eigen and geometry_msgs libraries.
 * The default Euler angle notation is the extrinsic RPY.
 *
 * @author Tomas Baca
 */

#ifndef ATTITUDE_CONVERTER_H
#define ATTITUDE_CONVERTER_H

#include <cmath>
#include <Eigen/Dense>
#include <tuple>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mrs_lib/geometry/misc.h>

namespace mrs_lib
{

// type of the object we are grasping
typedef enum
{

  RPY_INTRINSIC = 1,
  RPY_EXTRINSIC = 2,

} RPY_convention_t;

/* class EulerAttitude //{ */

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

//}

/* class Vector3Converter //{ */

/**
 * @brief Converter of Vector3 representations. Instantiate it with any type of vector3 in constructor and convert it by assigning it to any other type of
 * vector3 variable.
 */
class Vector3Converter {
public:
  /**
   * @brief Constructor with tf2::Vector3
   *
   * @param vector3
   */
  Vector3Converter(const tf2::Vector3& vector3) : vector3_(vector3){};

  /**
   * @brief Constructor with Eigen::Vector3
   *
   * @param vector3
   */
  Vector3Converter(const Eigen::Vector3d& vector3);

  /**
   * @brief Constructor with geometry_msgs::msg::Vector3
   *
   * @param vector3
   */
  Vector3Converter(const geometry_msgs::msg::Vector3& vector3);

  /**
   * @brief Constructor with doubles: x, y, z
   *
   * @param x
   * @param y
   * @param z
   */
  Vector3Converter(const double& x, const double& y, const double& z);

  /**
   * @brief typecast overloaded for tf2::Vector3
   *
   * @return vector3
   */
  operator tf2::Vector3() const;

  /**
   * @brief typecast overloaded for Eigen::Vector3
   *
   * @return vector3
   */
  operator Eigen::Vector3d() const;

  /**
   * @brief typecast overloaded for geometry_msgs::msg::Vector3
   *
   * @return vector3
   */
  operator geometry_msgs::msg::Vector3() const;

private:
  tf2::Vector3 vector3_;
};

//}

/**
 * @brief The main convertor class. Instantiate with any type in constructor and get the value in any other type by assigning the instance to your variable,
 * as: tf2::Quaternion tf2_quaternion = AttitudeConverter(roll, pitch, yaw); All the default Euler angles are in the extrinsic RPY notation.
 */
class AttitudeConverter {
public:
  /* exceptions //{ */

  //! is thrown when calculating of heading is not possible due to atan2 exception
  struct GetHeadingException : public std::exception
  {
    const char* what() const throw() {
      return "AttitudeConverter: can not calculate the heading, the rotated x-axis is parallel to the world's z-axis";
    }
  };

  //! is thrown when math breaks
  struct MathErrorException : public std::exception
  {
    const char* what() const throw() {
      return "AttitudeConverter: math error";
    }
  };

  //! is thrown when the internal attitude becomes invalid
  struct InvalidAttitudeException : public std::exception
  {
    const char* what() const throw() {
      return "AttitudeConverter: invalid attitude, the input probably constains NaNs";
    }
  };

  //! is thrown when the Euler angle format is set wrongly
  struct EulerFormatException : public std::exception
  {
    const char* what() const throw() {
      return "AttitudeConverter: invalid Euler angle format";
    }
  };

  //! is thrown when the heading cannot be set to an existing attitude
  struct SetHeadingException : public std::exception
  {
    const char* what() const throw() {
      return "AttitudeConverter: cannot set the desired heading, the thrust vector's Z component is 0";
    }
  };

  //}

  /* constructors //{ */

  /**
   * @brief Euler angles constructor
   *
   * @param roll
   * @param pitch
   * @param yaw
   * @param format optional, Euler angle convention, {"extrinsic", "intrinsic"}, defaults to "extrinsic"
   */
  AttitudeConverter(const double& roll, const double& pitch, const double& yaw, const RPY_convention_t& format = RPY_EXTRINSIC);

  /**
   * @brief geometry_msgs::msg::Quaternion constructor
   *
   * @param quaternion geometry_msgs::msg::Quaternion quaternion
   */
  AttitudeConverter(const geometry_msgs::msg::Quaternion quaternion);

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

  /**
   * @brief tf2::Matrix3x3 constructor
   *
   * @param quaternion tf2::Matrix3x3
   */
  AttitudeConverter(const tf2::Matrix3x3 matrix);

  //}

  /* operators //{ */

  /**
   * @brief typecast to tf2::Quaternion
   *
   * @return orientation in tf2::Quaternion
   */
  operator tf2::Quaternion() const;

  /**
   * @brief typecast to geometry_msgs::msg::Quaternion
   *
   * @return orientation in geometry_msgs::msg::Quaternion
   */
  operator geometry_msgs::msg::Quaternion() const;

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

  /**
   * @brief typecase to tf2::Matrix3x3
   *
   * @return tf2::Matrix3x3 rotational matrix
   */
  operator tf2::Matrix3x3() const;

  /**
   * @brief typecase to tf2::Transform
   *
   * @return tf2::Transform
   */
  operator tf2::Transform() const;

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

  /**
   * @brief get the angle of the rotated x-axis in the original XY plane, a.k.a
   *
   * @return heading
   */
  double getHeading(void);

  /**
   * @brief get heading rate base on the orientation and body-based attitude rate
   *
   * @param attitude_rate in the body frame
   *
   * @return heading rate in the world
   */
  double getHeadingRate(const Vector3Converter& attitude_rate);

  /**
   * @brief get the intrinsic yaw rate from a heading rate
   *
   * @param heading_rate
   *
   * @return intrinsic yaw rate
   */
  double getYawRateIntrinsic(const double& heading_rate);

  /**
   * @brief get a unit vector pointing in the X direction
   *
   * @return the vector
   */
  Vector3Converter getVectorX(void);

  /**
   * @brief get a unit vector pointing in the Y direction
   *
   * @return the vector
   */
  Vector3Converter getVectorY(void);

  /**
   * @brief get a unit vector pointing in the Z direction
   *
   * @return the vector
   */
  Vector3Converter getVectorZ(void);

  /**
   * @brief get the Roll, Pitch, Yaw angles in the Intrinsic convention
   *
   * @return RPY
   */
  std::tuple<double, double, double> getIntrinsicRPY();

  /**
   * @brief get the Roll, Pitch, Yaw angles in the Extrinsic convention. The same as the default AttitudeConverter assignment.
   *
   * @return RPY
   */
  std::tuple<double, double, double> getExtrinsicRPY();

  //}

  /* setters //{ */

  /**
   * @brief Updates the heading of the current orientation by updating the intrinsic yaw
   *
   * @param new heading
   *
   * @return the orientation
   */
  AttitudeConverter setHeading(const double& heading);

  /**
   * @brief Updates the extrinsic yaw of the current orientation.
   *
   * @param new yaw
   *
   * @return the orientation
   */
  AttitudeConverter setYaw(const double& new_yaw);

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
   * @brief throws exception when the internal attitude is invalid
   */
  void validateOrientation(void);

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
{
  static constexpr int value = 3;
};

template <>
struct std::tuple_element<0, mrs_lib::AttitudeConverter>
{
  using type = double;
};

template <>
struct std::tuple_element<1, mrs_lib::AttitudeConverter>
{
  using type = double;
};

template <>
struct std::tuple_element<2, mrs_lib::AttitudeConverter>
{
  using type = double;
};

#endif
