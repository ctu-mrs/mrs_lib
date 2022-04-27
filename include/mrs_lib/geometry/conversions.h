#ifndef GEOMETRY_CONVERSIONS_H
#define GEOMETRY_CONVERSIONS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

namespace mrs_lib
{
  namespace geometry
  {

    /* conversions from/to Eigen //{ */
    
    geometry_msgs::Point fromEigen(const Eigen::Vector3d& what)
    {
      geometry_msgs::Point pt;
      pt.x = what.x();
      pt.y = what.y();
      pt.z = what.z();
      return pt;
    }
    
    Eigen::Vector3d toEigen(const geometry_msgs::Point& what)
    {
      return {what.x, what.y, what.z};
    }
    
    Eigen::Vector3d toEigen(const geometry_msgs::Vector3& what)
    {
      return {what.x, what.y, what.z};
    }
    
    Eigen::Matrix<double, 6, 6> toEigenMatrix(const boost::array<double, 36>& what)
    {
      Eigen::Matrix<double, 6, 6> ret;
      for (int r = 0; r < 6; r++)
        for (int c = 0; c < 6; c++)
          ret(r, c) = what.at(6 * r + c);
      return ret;
    }
    
    geometry_msgs::Quaternion fromEigen(const Eigen::Quaterniond& what)
    {
      geometry_msgs::Quaternion q;
      q.x = what.x();
      q.y = what.y();
      q.z = what.z();
      q.w = what.w();
      return q;
    }
    
    Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& what)
    {
      // better to do this manually than through the constructor to avoid ambiguities (e.g. position of x and w)
      Eigen::Quaterniond q;
      q.x() = what.x;
      q.y() = what.y;
      q.z() = what.z;
      q.w() = what.w;
      return q;
    }
    
    //}

    /* conversions from/to OpenCV //{ */
    
    template <typename T>
    geometry_msgs::Point fromCV(const cv::Point3_<T>& what)
    {
      geometry_msgs::Point pt;
      pt.x = what.x;
      pt.y = what.y;
      pt.z = what.z;
      return pt;
    }
    
    template <typename T>
    cv::Point3_<T> toCV(const geometry_msgs::Point& what)
    {
      return {what.x, what.y, what.z};
    }
    
    template <typename T>
    cv::Point3_<T> toCV(const geometry_msgs::Vector3& what)
    {
      return {what.x, what.y, what.z};
    }
    
    //}

  }
}

#endif // GEOMETRY_CONVERSIONS_H
