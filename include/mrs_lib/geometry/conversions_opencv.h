#ifndef GEOMETRY_CONVERSIONS_OPENCV_H
#define GEOMETRY_CONVERSIONS_OPENCV_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <opencv2/core/types.hpp>

namespace mrs_lib
{
  namespace geometry
  {

    /* conversions from/to OpenCV //{ */
    
    geometry_msgs::Point fromCV(const cv::Point3d& what);
    
    cv::Point3d toCV(const geometry_msgs::Point& what);
    
    cv::Point3d toCV(const geometry_msgs::Vector3& what);
    
    //}

  }
}

#endif // GEOMETRY_CONVERSIONS_OPENCV_H
