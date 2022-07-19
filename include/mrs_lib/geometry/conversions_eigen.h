#ifndef GEOMETRY_CONVERSIONS_EIGEN_H
#define GEOMETRY_CONVERSIONS_EIGEN_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mrs_lib
{
  namespace geometry
  {

    /* conversions from/to Eigen //{ */
    
    geometry_msgs::Point fromEigen(const Eigen::Vector3d& what);

    geometry_msgs::Vector3 fromEigenVec(const Eigen::Vector3d& what);
    
    Eigen::Vector3d toEigen(const geometry_msgs::Point& what);
    
    Eigen::Vector3d toEigen(const geometry_msgs::Vector3& what);
    
    Eigen::Matrix<double, 6, 6> toEigenMatrix(const boost::array<double, 36>& what);
    
    geometry_msgs::Quaternion fromEigen(const Eigen::Quaterniond& what);
    
    Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& what);
    
    //}

  }
}

#endif // GEOMETRY_CONVERSIONS_EIGEN_H
