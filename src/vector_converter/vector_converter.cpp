#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#include <mrs_lib/vector_converter.h>
#pragma GCC diagnostic pop

#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/imgproc/imgproc.hpp>

// Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
// Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

template Eigen::Vector3d mrs_lib::convert<Eigen::Vector3d>(const cv::Vec3d&);

/* using namespace mrs_lib; */

/* int main() */
/* { */
/*   const Eigen::Vector3d vec1; */

/*   const auto vec2 = convert<geometry_msgs::Vector3>(vec1); */
/*   const auto vec3 = convert<cv::Vec3d>(vec1); */
/*   const auto vec4 = convert<cv::Vec3f>(vec1); */
/*   const auto vec5 = convert<Eigen::Vector3d>(vec3); */
/*   const auto vec6 = convert<Eigen::Vector3f>(vec2); */

/*   std::cout << "vec1: " << vec1 << std::endl; */
/*   std::cout << "vec2: " << vec2 << std::endl; */
/*   std::cout << "vec3: " << vec3 << std::endl; */
/*   std::cout << "vec4: " << vec4 << std::endl; */
/*   std::cout << "vec5: " << vec5 << std::endl; */
/*   std::cout << "vec6: " << vec6 << std::endl; */

/*   return 0; */
/* }; */

