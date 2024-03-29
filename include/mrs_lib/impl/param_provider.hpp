#ifndef PARAM_PROVIDER_HPP
#define PARAM_PROVIDER_HPP

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
        {}
      }

    }

    if (m_use_rosparam)
      return m_nh.getParam(param_name, value_out);

    return false;
  }
}

#endif // PARAM_PROVIDER_HPP
