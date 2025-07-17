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
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& var)
  {
    for (size_t it = 0; it < var.size(); it++)
    {
      os << var.at(it);
      if (it < var.size() - 1)
        os << ", ";
    }
    return os;
  }

  template <typename Key, typename Value>
  std::ostream& operator<<(std::ostream& os, const std::map<Key, Value>& var)
  {
    size_t it = 0;
    for (const auto& pair : var) {
      os << pair.first << ": " << pair.second;
      if (it < var.size() - 1)
        os << std::endl;
      it++;
    }
    return os;
  }

  /* ParamProvider::resolved_name_t //{ */
  
  struct ParamProvider::resolved_name_t
  {
    std::string str;

    resolved_name_t() = default;
    resolved_name_t(std::string&& s) : str(s) {}
    resolved_name_t(const std::string& s) : str(s) {}

    // Explicit conversion
    explicit operator std::string() const
    {
      return str;
    }

    friend std::ostream& operator<<(std::ostream& os, const resolved_name_t& r)
    {
      return os << r.str;
    }

    friend bool operator==(const resolved_name_t& lhs, const resolved_name_t& rhs)
    {
      return lhs.str == rhs.str;
    }
  };
  
  //}

  /* to_param_type() method //{ */
  template <typename T>
  rclcpp::ParameterType to_param_type()
  {
    return rclcpp::ParameterValue{T{}}.get_type();
  }
  //}

  /* getParam() method and overloads //{ */
  template <typename T>
  bool ParamProvider::getParam(const std::string& param_name, T& value_out) const
  {
    return getParam(resolveName(param_name), value_out, {});
  }

  template <typename T>
  bool ParamProvider::getParam(const resolved_name_t& resolved_name, T& value_out, const get_options_t<T>& opts) const
  {
    bool use_rosparam = m_use_rosparam;
    // the options structure always has precedence if the parameter is set
    if (opts.use_rosparam.has_value())
      use_rosparam = opts.use_rosparam.value();

    // first, try to load from YAML, if enabled
    if (opts.use_yaml && loadFromYaml(resolved_name, value_out, opts))
      return true;

    // then, try to load from ROS, if enabled
    if (use_rosparam && loadFromROS(resolved_name, value_out, opts))
      return true;

    // if both fail, check for a default value
    if (opts.declare_options.default_value.has_value())
    {
      const auto& default_value = opts.declare_options.default_value.value();

      if (opts.always_declare && !declareParam<T>(resolved_name, opts.declare_options))
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to declare parameter \"" << resolved_name << "\".");
        return false;
      }

      value_out = default_value;
      return true;
    }

    // if all options fail, return false
    std::stringstream ss;
    ss << "Param '" << resolved_name << "' not found:";
    if (opts.use_yaml)
      ss << " in YAML files,";
    if (use_rosparam)
      ss << " in ROS,";
    ss << " no default value provided.";
    RCLCPP_ERROR_STREAM(m_node->get_logger(), ss.str());
    return false;
  }
  //}

  /* setParam() method //{ */
  template <typename T>
  bool ParamProvider::setParam(const std::string& param_name, const T& value) const
  {
    return setParam(resolveName(param_name), value);
  }

  template <typename T>
  bool ParamProvider::setParam(const resolved_name_t& resolved_name, const T& value, const set_options_t<T>& opts) const
  {
    // if the parameter is not yet declared, declare and set it together
    if (!m_node->has_parameter(resolved_name.str))
    {
      auto declare_opts_local = opts.declare_options;
      declare_opts_local.default_value = value;
      const auto result = declareParam<T>(resolved_name, declare_opts_local);
      if (!result)
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not declare and set param '" << resolved_name << "'!");
      return result;
    }

    // otherwise, try setting it
    try
    {
      /* RCLCPP_INFO_STREAM(m_node->get_logger(), "Setting param '" << resolved_name << "'"); */
      rcl_interfaces::msg::SetParametersResult res = m_node->set_parameter({resolved_name.str, value});
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
  //}

  /* declareParam() method and overloads //{ */
  template <typename T>
  bool ParamProvider::declareParam(const std::string& param_name) const
  {
    return declareParam<T>(resolveName(param_name), {});
  }

  template <typename T>
  bool ParamProvider::declareParam(const std::string& param_name, const T& default_value) const
  {
    return declareParam<T>(resolveName(param_name), {.default_value = default_value});
  }

  template <typename T>
  bool ParamProvider::declareParam(const resolved_name_t& resolved_name, const declare_options_t<T>& opts) const
  {
    try
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = !opts.reconfigurable;

      // if the parameter range is specified, set it if applicable
      if (opts.range.has_value())
      {
        const auto& opt_range = opts.range.value();
        // set the range approprately according to the parameter type
        if constexpr (!numeric<T>)
        {
          // if the type is not numerical, print an error to let the user know and fail
          RCLCPP_ERROR_STREAM(m_node->get_logger(), "Error when declaring parameter \"" << resolved_name << "\": Range cannot be set for non-numerical values! Ignoring range.");
          return false;
        }
        else if constexpr (std::integral<T>)
        {
          // integer range for integral types
          rcl_interfaces::msg::IntegerRange range;
          range.from_value = opt_range.minimum;
          range.to_value = opt_range.maximum;
          descriptor.integer_range.push_back(range);
        }
        else if constexpr (std::floating_point<T>)
        {
          // floating-point range for floating-point types
          rcl_interfaces::msg::FloatingPointRange range;
          range.from_value = opt_range.minimum;
          range.to_value = opt_range.maximum;
          descriptor.floating_point_range.push_back(range);
        }
      }

      // actually declare the parameter
      if (opts.default_value.has_value())
      {
        m_node->declare_parameter(resolved_name.str, opts.default_value.value(), descriptor);
      }
      else
      {
        // a hack to allow declaring a strong-typed parameter without a default value
        // because ROS2 apparently fell on its head when it was a litle baby
        // this is actually not documented in the API documentation, so see https://github.com/ros2/rclcpp/issues/1691
        const rclcpp::ParameterType type = to_param_type<T>();
        descriptor.type = type;
        m_node->declare_parameter(resolved_name.str, type, descriptor);
      }
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
  //}

  /* loadFromYaml() method //{ */
  template <typename T>
  bool ParamProvider::loadFromYaml(const resolved_name_t& resolved_name, T& value_out, const get_options_t<T>& opts) const
  {
    T loaded_value;
  
    const auto found_node = findYamlNode(resolved_name);
    if (!found_node.has_value())
    {
      return false;
    }
  
    try
    {
      // try catch is the only type-generic option...
      loaded_value = found_node.value().as<T>();
    }
    catch (const YAML::BadConversion& e)
    {
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: The YAML-loaded parameter has a wrong type: " << e.what());
      return false;
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: YAML-CPP threw an unknown exception: " << e.what());
      return false;
    }
  
    // declare the parameter if the options specify to always define it
    if (opts.always_declare)
    {
      auto declare_opts_local = opts.declare_options;
      declare_opts_local.default_value = loaded_value;
  
      // see https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html#parameters
      if (!m_node->has_parameter(resolved_name.str) && !declareParam<T>(resolved_name, declare_opts_local))
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to declare parameter \"" << resolved_name << "\".");
        return false;
      }
    }
  
    // if all is OK, set the output value
    value_out = loaded_value;
    // the parameter value was successfully loaded and the parameter was declared if required, everything is done, return true
    return true;
  }
  //}

  /* ranges_match() method //{ */
  template <typename T>
  bool ParamProvider::ranges_match(const rcl_interfaces::msg::ParameterDescriptor& descriptor, const range_t<T>& declare_range) const
  {
    if constexpr (numeric<T>)
    {
      if (!descriptor.floating_point_range.empty())
      {
        const auto& desc_range = descriptor.floating_point_range.front();
        if (desc_range.from_value != declare_range.minimum || desc_range.to_value != declare_range.maximum)
          return false;
      }
  
      if (!descriptor.integer_range.empty())
      {
        const auto& desc_range = descriptor.integer_range.front();
        if (desc_range.from_value != declare_range.minimum || desc_range.to_value != declare_range.maximum)
          return false;
      }
    }

    // trying to specify a range for a non-numeric type...
    return false;
  }

//}

  /* loadFromROS() method //{ */
  template <typename T>
  bool ParamProvider::loadFromROS(const resolved_name_t& resolved_name, T& value_out, const get_options_t<T>& opts) const
  {
    std::optional<T> loaded_value;
    const bool was_declared = m_node->has_parameter(resolved_name.str);
  
    // check if the current declaration is compatible with the desired declaration
    if (was_declared)
    {
      const auto descriptor = m_node->describe_parameter(resolved_name.str);
      if (descriptor.read_only && opts.declare_options.reconfigurable)
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Parameter \"" << resolved_name << "\" already declared as read-only, cannot re-declare as reconfigurable!");
        return false;
      }
  
      if (opts.declare_options.range.has_value() && !ranges_match(descriptor, opts.declare_options.range.value()))
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Parameter \"" << resolved_name << "\" already declared with a range, cannot re-declare with a different one!");
        return false;
      }
    }
  
    // if the parameter is not declared yet, check if it is available in ROS by declaring it as dynamically-typed and read-writable
    if (!was_declared)
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = false;
      descriptor.dynamic_typing = true;
      try
      {
        m_node->declare_parameter(resolved_name.str, rclcpp::ParameterValue(), descriptor);
      }
      catch (const std::exception& e)
      {
        // if the declaration already fails, something is wrong
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to declare parameter \"" << resolved_name << "\": " << e.what());
        return false;
      }
    }
  
    // now try loading the parameter
    try
    {
      /* RCLCPP_INFO_STREAM(m_node->get_logger(), "Getting param '" << resolved_name << "'"); */
      rclcpp::Parameter param;
      if (m_node->get_parameter(resolved_name.str, param))
        loaded_value = param.get_value<T>();
    }
    // if the parameter has a wrong value, return failure
    catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
    {
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "Could not get param '" << resolved_name << "' from ROS: " << e.what());
    }
  
    // if the parameter was not declared before, undecalre it:
    // 1. either it was loaded successfully and has to be re-declared with the correct options
    // 2. or it was not loaded and has to be un-declared
    if (!was_declared)
      m_node->undeclare_parameter(resolved_name.str);
  
    // loading from ROS was not successful, just return false
    if (!loaded_value.has_value())
      return false;
  
    // if the parameter was not declared before, redeclare it with the correct parameters
    if (!was_declared)
    {
      auto declare_opts_local = opts.declare_options;
      declare_opts_local.default_value = value_out;
      if (!declareParam<T>(resolved_name, declare_opts_local))
      {
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed to declare parameter \"" << resolved_name << "\".");
        return false;
      }
    }
  
    // if all went good, set the value and return true
    value_out = std::move(loaded_value.value());
    return true;
  }
  //}

}  // namespace mrs_lib

namespace std
{
  template <>
  struct hash<mrs_lib::ParamProvider::resolved_name_t>
  {
    std::size_t operator()(const mrs_lib::ParamProvider::resolved_name_t& r) const noexcept
    {
      return std::hash<std::string>{}(r.str);
    }
  };
}
