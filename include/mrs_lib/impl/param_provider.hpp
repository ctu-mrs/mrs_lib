#ifndef PARAM_PROVIDER_HPP
#define PARAM_PROVIDER_HPP

#ifndef PARAM_PROVIDER_H
#include <mrs_lib/param_provider.h>
#endif

namespace mrs_lib
{
template <typename T>
bool ParamProvider::getParam(const std::string& param_name, T& value_out) const {
  try {
    return getParamImpl(param_name, value_out);
  }
  catch (const YAML::Exception& e) {
    RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: YAML-CPP threw an unknown exception: " << e.what());
    return false;
  }
}

template <typename T>
bool ParamProvider::getParamImpl(const std::string& param_name, T& value_out) const {
  {
    const auto found_node = findYamlNode(param_name);
    if (found_node.has_value()) {
      try {
        // try catch is the only type-generic option...
        value_out = found_node.value().as<T>();
        return true;
      }
      catch (const YAML::BadConversion& e) {
      }
    }
  }

  if (m_use_rosparam) {

    auto reformated_name = param_name;
    // reformat all '/' to '.' which is the new ROS2 delimeter
    std::replace(reformated_name.begin(), reformated_name.end(), '/', '.');

    // firstly, the parameter has to be declared
    // see https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html#parameters
    if (!m_node->has_parameter(reformated_name)) {
      try {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;
        descriptor.read_only      = true;

        // loading as a rclcpp::ParameterValue() allows for dynamic typing (may need removal if this leads to higher memory/computation load)
        m_node->declare_parameter(reformated_name, rclcpp::ParameterValue(), descriptor);
      }
      catch (const std::exception& e) {
        // this can happen if (see
        // http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb):
        // * parameter has already been declared              (rclcpp::exceptions::ParameterAlreadyDeclaredException)
        // * parameter name is invalid                        (rclcpp::exceptions::InvalidParametersException)
        // * initial value fails to be set                    (rclcpp::exceptions::InvalidParameterValueException, not sure what exactly this means)
        // * type of the default value or override is wrong   (rclcpp::exceptions::InvalidParameterTypeException, the most common one)
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not declare param '" << reformated_name << "': " << e.what());

        return false;
      }
    }

    rclcpp::Parameter tmp{};
    bool success = m_node->get_parameter(reformated_name, tmp);
    if (success) {
      value_out = tmp.get_value<T>();
    }
    
    return success;
  }

  return false;
}
}  // namespace mrs_lib

#endif  // PARAM_PROVIDER_HPP
