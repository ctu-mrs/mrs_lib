#include <Eigen/Dense>
#include <tuple>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace mrs_lib
{

  template <typename in_t>
  std::tuple<double, double, double> convertFrom(const in_t& in)
  {
    return {in.x, in.y, in.z};
  }

  template <>
  std::tuple<double, double, double> convertFrom<Eigen::Vector3d>(const Eigen::Vector3d& in)
  {
    return {in.x(), in.y(), in.z()};
  }

  template <>
  std::tuple<double, double, double> convertFrom<cv::Vec3d>(const cv::Vec3d& in)
  {
    return {in[0], in[1], in[2]};
  }

  template <typename ret_t>
  ret_t convertTo(const double x, const double y, const double z)
  {
    return ret_t {x, y, z};
  }

  template <>
  geometry_msgs::Vector3 convertTo<geometry_msgs::Vector3>(const double x, const double y, const double z)
  {
    geometry_msgs::Vector3 ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
  }

  template <typename ret_t, typename in_t>
  ret_t convert(const in_t& in)
  {
    const auto [x, y, z] = convertFrom(in);
    return convertTo<ret_t>(x, y, z);
  }

}
