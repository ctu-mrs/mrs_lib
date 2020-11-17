#include <mrs_lib/vector_converter.h>

using namespace mrs_lib;

int main()
{
  const Eigen::Vector3d vec1;

  const auto vec2 = convert<geometry_msgs::Vector3>(vec1);
  const auto vec3 = convert<cv::Vec3d>(vec1);

  return 0;
};
