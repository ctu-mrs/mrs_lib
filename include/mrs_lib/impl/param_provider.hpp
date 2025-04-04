// clang: MatousFormat
/**  \file
     \brief Implements ParamProvider - a convenience class for seamlessly loading parameters from YAML or ROS.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Afzal Ahmad
 */
#pragma once

#include <mrs_lib/param_provider.h>

namespace mrs_lib
{

  template <typename T>
  bool ParamProvider::getParam(const std::string& param_name, T& value_out, const bool reconfigurable) const
  {
    const auto found_node = findYamlNode(param_name);
    if (found_node.has_value())
    {
      try
      {
        // try catch is the only type-generic option...
        value_out = found_node.value().as<T>();
        return true;
      }
      catch (const YAML::BadConversion& e)
      {
      }
<<<<<<< HEAD
      catch (const YAML::Exception& e)
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: YAML-CPP threw an unknown exception: " << e.what());
=======
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

>>>>>>> ros2_devel
        return false;
      }
    }

    if (m_use_rosparam)
    {
      const auto resolved_name = resolveName(param_name);
      // firstly, the parameter has to be declared
      // see https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html#parameters
      if (!m_node->has_parameter(resolved_name) && declareParam<T>(param_name, reconfigurable))
        return false;

      rclcpp::Parameter tmp{};
      bool success = m_node->get_parameter(resolved_name, tmp);
      if (success)
      {
        try
        {
          value_out = tmp.get_value<T>();
        }
        // if the parameter has a wrong value, return failure
        catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
        {
          return false;
        }
      }

      return success;
    }

    return false;
  }

  template <typename T>
  bool ParamProvider::declareParam(const std::string& param_name, const bool reconfigurable) const
  {
    const auto resolved_name = resolveName(param_name);
    try
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = !reconfigurable;
      descriptor.dynamic_typing = true; // this is necessary because new ROS2 does not allow statically typed parameters without a default value... why???
      m_node->declare_parameter(resolved_name, rclcpp::ParameterValue{}, descriptor);
    }
    catch (const std::exception& e)
    {
      // this can happen if (see
      // http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb):
      // * parameter has already been declared              (rclcpp::exceptions::ParameterAlreadyDeclaredException)
      // * parameter name is invalid                        (rclcpp::exceptions::InvalidParametersException)
      // * initial value fails to be set                    (rclcpp::exceptions::InvalidParameterValueException, not sure what exactly this means)
      // * type of the default value or override is wrong   (rclcpp::exceptions::InvalidParameterTypeException, the most common one)
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not declare param '" << resolved_name << "': " << e.what());

      return false;
    }
    return true;
  }

  template <typename T>
  bool ParamProvider::declareParamDefault(const std::string& param_name, const T& default_value, const bool reconfigurable) const
  {
    const auto resolved_name = resolveName(param_name);
    try
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = !reconfigurable;
      descriptor.dynamic_typing = false;
      m_node->declare_parameter(resolved_name, default_value, descriptor);
    }
    catch (const std::exception& e)
    {
      // this can happen if (see
      // http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb):
      // * parameter has already been declared              (rclcpp::exceptions::ParameterAlreadyDeclaredException)
      // * parameter name is invalid                        (rclcpp::exceptions::InvalidParametersException)
      // * initial value fails to be set                    (rclcpp::exceptions::InvalidParameterValueException, not sure what exactly this means)
      // * type of the default value or override is wrong   (rclcpp::exceptions::InvalidParameterTypeException, the most common one)
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not declare param '" << resolved_name << "': " << e.what());

      return false;
    }
    return true;
  }
}  // namespace mrs_lib
