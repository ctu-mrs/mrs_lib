// clang: MatousFormat
#ifndef GEOMETRY_CONVERSIONS_EIGEN_H
#define GEOMETRY_CONVERSIONS_EIGEN_H

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mrs_lib
{
  namespace geometry
  {

    /* conversions from/to Eigen //{ */

    geometry_msgs::msg::Point fromEigen(const Eigen::Vector3d& what);

    geometry_msgs::msg::Vector3 fromEigenVec(const Eigen::Vector3d& what);

    Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& what);

    Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& what);

    Eigen::Matrix<double, 6, 6> toEigenMatrix(const std::array<double, 36>& what);

    geometry_msgs::msg::Quaternion fromEigen(const Eigen::Quaterniond& what);

    Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion& what);

    //}

  }  // namespace geometry
}  // namespace mrs_lib

#endif  // GEOMETRY_CONVERSIONS_EIGEN_H
