// Include necessary files for the vector types here
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>

// Add the types for which you want the conversion to be instantiated to this list
#define VC_TYPE_LIST Eigen::Vector3d, Eigen::Vector3f, cv::Vec3d, cv::Vec3f, geometry_msgs::Vector3, pcl::PointXYZ
