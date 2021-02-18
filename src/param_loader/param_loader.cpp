#include <limits>
#include <mrs_lib/param_loader.h>
#include <rcl_interfaces/msg/detail/integer_range__struct.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rosidl_runtime_cpp/bounded_vector.hpp>

rcl_interfaces::msg::ParameterDescriptor mrs_lib::describe_param(
    const std::string& description,
    const std::string& additional_constraints
    )
{
  rcl_interfaces::msg::ParameterDescriptor ret;

  ret.description = description;
  ret.additional_constraints = additional_constraints;

  return ret;
}

rcl_interfaces::msg::ParameterDescriptor mrs_lib::describe_param(
    const int range_from,
    const int range_to,
    const int range_step,
    const std::string& description,
    const std::string& additional_constraints
    )
{
  rcl_interfaces::msg::ParameterDescriptor ret;

  rcl_interfaces::msg::IntegerRange range;
  range.from_value = range_from;
  range.to_value = range_to;
  range.step = range_step;
  ret.integer_range.push_back(range);
  ret.description = description;
  ret.additional_constraints = additional_constraints;
  ret.type = rclcpp::PARAMETER_DOUBLE;

  return ret;
}

rcl_interfaces::msg::ParameterDescriptor mrs_lib::describe_param(
    const double range_from,
    const double range_to,
    const double range_step,
    const std::string& description,
    const std::string& additional_constraints
    )
{
  rcl_interfaces::msg::ParameterDescriptor ret;

  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = range_from;
  range.to_value = range_to;
  range.step = range_step;
  ret.floating_point_range.push_back(range);
  ret.description = description;
  ret.additional_constraints = additional_constraints;
  ret.type = rclcpp::PARAMETER_INTEGER;

  return ret;
}

// Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
// Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

/* template bool mrs_lib::ParamLoader::load_param2<bool>(const std::string& name, const bool& default_value); */

/* template int mrs_lib::ParamLoader::load_param2<int>(const std::string& name, const int& default_value); */

/* template double mrs_lib::ParamLoader::load_param2<double>(const std::string& name, const double& default_value); */

/* template std::string mrs_lib::ParamLoader::load_param2<std::string>(const std::string& name, const std::string& default_value); */

