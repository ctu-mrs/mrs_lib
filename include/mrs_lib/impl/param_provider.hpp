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
  rclcpp::ParameterType to_param_type()
  {
    return rclcpp::ParameterValue{T{}}.get_type();
  }

  template <typename T>
  bool ParamProvider::getParam(const std::string& param_name, T& value_out, const bool reconfigurable) const
  {
    return getParam(resolveName(param_name), value_out, reconfigurable);
  }

  template <typename T>
  bool ParamProvider::getParam(const resolved_name_t& resolved_name, T& value_out, const bool reconfigurable) const
  {
    const auto found_node = findYamlNode(resolved_name);
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
      catch (const YAML::Exception& e)
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: YAML-CPP threw an unknown exception: " << e.what());
        return false;
      }
    }

    if (m_use_rosparam)
    {
      // firstly, the parameter has to be declared
      // see https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html#parameters
      if (!m_node->has_parameter(resolved_name) && !declareParam<T>(resolved_name, reconfigurable))
        return false;

      try
      {
        /* RCLCPP_INFO_STREAM(m_node->get_logger(), "Getting param '" << resolved_name << "'"); */
        rclcpp::Parameter param;
        if (!m_node->get_parameter(resolved_name, param))
        {
          // do not print an error as the parameter may have been optional - it is therefore OK if it is not declared
          /* RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not get param '" << resolved_name << "' (not declared)"); */
          return false;
        }
        value_out = param.get_value<T>();
      }
      // if the parameter has a wrong value, return failure
      catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not get param '" << resolved_name << "': " << e.what());
        return false;
      }

      return true;
    }

    RCLCPP_ERROR_STREAM(m_node->get_logger(), "Param '" << resolved_name << "' not found in YAML files");
    return false;
  }

  template <typename T>
  bool ParamProvider::setParam(const std::string& param_name, const T& value, const bool reconfigurable) const
  {
    if (m_use_rosparam)
    {
      const auto resolved_name = resolveName(param_name);
      // firstly, the parameter has to be declared
      // see https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html#parameters
      if (!m_node->has_parameter(resolved_name) && !declareParam<T>(param_name, reconfigurable))
        return false;

      try
      {
        /* RCLCPP_INFO_STREAM(m_node->get_logger(), "Setting param '" << resolved_name << "'"); */
        rcl_interfaces::msg::SetParametersResult res = m_node->set_parameter({resolved_name, value});
        if (!res.successful)
        {
          RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not set param '" << resolved_name << "': " << res.reason);
          return false;
        }
      }
      // if the parameter has a wrong value, return failure
      catch (const rclcpp::exceptions::ParameterNotDeclaredException& e)
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not set param '" << resolved_name << "': " << e.what());
        return false;
      }

      return true;
    }

    RCLCPP_ERROR_STREAM(m_node->get_logger(), "use_rosparam is false - cannot set YAML parameter value!");
    return false;
  }

  template <typename T>
  bool ParamProvider::declareParam(const std::string& param_name, const bool reconfigurable) const
  {
    return declareParam<T>(resolveName(param_name), reconfigurable);
  }

  template <typename T>
  bool ParamProvider::declareParam(const ParamProvider::resolved_name_t& resolved_name, const bool reconfigurable) const
  {
    try
    {
      // a hack to allow declaring a strong-typed parameter without a default value
      // because ROS2 apparently fell on its head when it was a litle baby
      // this is actually not documented in the API documentation, so see https://github.com/ros2/rclcpp/issues/1691
      const rclcpp::ParameterType type = to_param_type<T>();
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = !reconfigurable;
      descriptor.type = type;
      /* RCLCPP_INFO_STREAM(m_node->get_logger(), "Declaring param '" << resolved_name << "' of type " << type); */
      const rclcpp::ParameterValue ret = m_node->declare_parameter(resolved_name, type, descriptor);
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
