#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <mrs_lib/param_loader.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr nh = std::make_shared<rclcpp::Node>("param_loader_example");

  rcl_interfaces::msg::ParameterDescriptor descr = mrs_lib::describe_param(0.0, 1.0, 0.01, "kP of throttle PID", "don't set to nonsense");
/*   rclcpp::ParameterValue defval(0.01); */
/*   nh->declare_parameter("param_namespace.floating_number", defval, descr); */
/*   auto param = nh->get_parameter("param_namespace.floating_number"); */
/*   std::cout << param << std::endl; */

  mrs_lib::ParamLoader pl(nh);

  double      floating_point_number;
  std::string some_string;
  pl.load_param_dynamic("param_namespace.floating_number", floating_point_number, 0.01, descr);
  pl.load_param_reusable("param_namespace.floating_number", floating_point_number);
  pl.load_param("some_string", some_string, "this is the default value of the some_string parameter");
  const auto uav_type = pl.load_param2<std::string>("uav_type");

  if (!pl.loaded_successfully())
  {
    RCLCPP_ERROR(nh->get_logger(), "Some compulsory parameters were not loaded! Ending node.");
    rclcpp::shutdown(nullptr, "Some compulsory parameters were not loaded");
    return 1;
  }

  pl.enable_callbacks();

  rclcpp::spin(nh);
  rclcpp::shutdown();

  return 0;
}  // namespace ros2_examples

