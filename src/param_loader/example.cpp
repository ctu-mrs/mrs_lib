#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <mrs_lib/param_loader.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr nh = std::make_shared<rclcpp::Node>("ParamLoader_example");
  mrs_lib::ParamLoader pl(*nh);

  double      floating_point_number;
  std::string some_string;

  pl.load_param("param_namespace.floating_number", floating_point_number, true);
  pl.load_param("some_string", some_string);
  const auto uav_type = pl.load_param2<std::string>("uav_type");

  /* if (!pl.loaded_successfully()) */
  /* { */
  /*   RCLCPP_ERROR(nh->get_logger(), "Some compulsory parameters were not loaded! Ending node."); */
  /*   rclcpp::shutdown(nullptr, "Some compulsory parameters were not loaded"); */
  /*   return 1; */
  /* } */

  rclcpp::spin(nh);
  rclcpp::shutdown();

  return 0;
}  // namespace ros2_examples

