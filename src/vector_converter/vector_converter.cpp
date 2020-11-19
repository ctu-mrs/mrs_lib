#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#include <mrs_lib/vector_converter.h>
#pragma GCC diagnostic pop

using namespace mrs_lib;

int main()
{
  const Eigen::Vector3d vec1;

  const auto vec2 = convert<geometry_msgs::Vector3>(vec1);
  const auto vec3 = convert<cv::Vec3d>(vec1);
  const auto vec4 = convert<cv::Vec3f>(vec1);
  const auto vec5 = convert<Eigen::Vector3d>(vec3);
  const auto vec6 = convert<Eigen::Vector3f>(vec2);

  std::cout << "vec1: " << vec1 << std::endl;
  std::cout << "vec2: " << vec2 << std::endl;
  std::cout << "vec3: " << vec3 << std::endl;
  std::cout << "vec4: " << vec4 << std::endl;
  std::cout << "vec5: " << vec5 << std::endl;
  std::cout << "vec6: " << vec6 << std::endl;

  return 0;
};

