#include <chrono>
#include <mrs_lib/param_loader.h>

// Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
// Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

template bool mrs_lib::ParamLoader::loadParam<bool>(const std::string& name, bool& out_value, const bool& default_value);
template bool mrs_lib::ParamLoader::loadParamReusable<bool>(const std::string& name, bool& out_value, const bool& default_value);

template bool mrs_lib::ParamLoader::loadParam<int>(const std::string& name, int& out_value, const int& default_value);
template bool mrs_lib::ParamLoader::loadParamReusable<int>(const std::string& name, int& out_value, const int& default_value);

template bool mrs_lib::ParamLoader::loadParam<double>(const std::string& name, double& out_value, const double& default_value);
template bool mrs_lib::ParamLoader::loadParamReusable<double>(const std::string& name, double& out_value, const double& default_value);

template bool mrs_lib::ParamLoader::loadParam<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);
template bool mrs_lib::ParamLoader::loadParamReusable<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);

template <>
rclcpp::Duration mrs_lib::ParamLoader::loadParam2<rclcpp::Duration>(const std::string& name, const rclcpp::Duration& default_value)
{
  const auto secs = loadParam2<double>(name, default_value.seconds());
  return rclcpp::Duration::from_seconds(secs);
}

template <>
rclcpp::Duration mrs_lib::ParamLoader::loadParam2<rclcpp::Duration>(const std::string& name)
{
  const auto secs = loadParam2<double>(name);
  return rclcpp::Duration::from_seconds(secs);
}
