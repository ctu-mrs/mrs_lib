#pragma once

#include <mrs_lib/param_provider.h>

namespace mrs_lib
{
  template <typename T>
  bool ParamProvider::getParam(const std::string& param_name, T& value_out) const
  {
    try
    {
      return getParamImpl(param_name, value_out);
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_STREAM("[" << m_node_name << "]: YAML-CPP threw an unknown exception: " << e.what());
      return false;
    }
  }

  template <typename T>
  bool ParamProvider::getParamImpl(const std::string& param_name, T& value_out) const
  {
    {
      const auto found_node_opt = findYamlNode(param_name);
      if (found_node_opt.has_value())
      {
        const auto& found_node = found_node_opt.value();
        try
        {
          // try catch is the only type-generic option...
          value_out = applyTag<T>(found_node);
          return true;
        }
        catch (const YAML::BadConversion& e)
        {}
      }

    }

    if (m_use_rosparam)
      return m_nh.getParam(param_name, value_out);

    return false;
  }

  template<typename T> 
  inline T ParamProvider::degrees2radians(const T degrees)
  {
    return degrees / T(180) * T(M_PI);
  }

  template<typename T>
  T ParamProvider::applyTag(const YAML::Node& node) const
  {
    if constexpr (std::is_floating_point_v<T>)
    {
      if (node.Tag() == "!degrees")
        return degrees2radians(node.as<T>());
    }
    return node.as<T>();
  }
}
