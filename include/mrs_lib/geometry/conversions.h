#ifndef GEOMETRY_CONVERSIONS_H
#define GEOMETRY_CONVERSIONS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

namespace mrs_lib
{
  namespace geometry
  {

    /* conversions from/to Eigen //{ */
    
    geometry_msgs::Point fromEigen(const Eigen::Vector3d& what);
    
    
    Eigen::Vector3d toEigen(const geometry_msgs::Point& what);
    
    
    Eigen::Vector3d toEigen(const geometry_msgs::Vector3& what);
    
    
    Eigen::Matrix<double, 6, 6> toEigenMatrix(const boost::array<double, 36>& what);
    
    
    geometry_msgs::Quaternion fromEigen(const Eigen::Quaterniond& what);
    
    
    Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& what);
    
    //}

    /* conversions from/to OpenCV //{ */
    
    geometry_msgs::Point fromCV(const cv::Point3d& what);
    
    cv::Point3d toCV(const geometry_msgs::Point& what);
    
    cv::Point3d toCV(const geometry_msgs::Vector3& what);
    
    //}

  }
}

#endif // GEOMETRY_CONVERSIONS_H
