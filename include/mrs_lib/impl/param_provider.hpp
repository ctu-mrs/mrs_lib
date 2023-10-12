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
    for (const auto& yaml : m_yamls)
    {
      // Try to load the parameter sequentially as a map.
      auto cur_node_it = std::cbegin(yaml);
      // The root should always be a pam
      if (!cur_node_it->second.IsMap())
        continue;

      bool loaded = true;
      {
        constexpr char delimiter = '/';
        const std::string_view param_name_sw(param_name);
        auto substr_start = std::cbegin(param_name_sw);
        auto substr_end = substr_start;
        do
        {
          substr_end = std::find(substr_start, std::cend(param_name_sw), delimiter);
          // why can't substr or string_view take iterators? :'(
          const auto start_pos = std::distance(std::cbegin(param_name_sw), substr_start);
          const auto count = std::distance(substr_start, substr_end);
          const std::string_view param_substr = param_name_sw.substr(start_pos, count);
          substr_start = substr_end+1;

          bool found = false;
          for (auto node_it = std::cbegin(cur_node_it->second); node_it != std::cend(cur_node_it->second); ++node_it)
          {
            const std::string_view node_name_sw(node_it->first.as<std::string>());
            if (node_name_sw == param_substr)
            {
              cur_node_it = node_it;
              found = true;
              break;
            }
          }

          if (!found)
          {
            loaded = false;
            break;
          }
        }
        while (substr_end != std::end(param_name_sw) && cur_node_it->second.IsMap());
      }

      if (loaded)
      {
        try
        {
          // try catch is the only type-generic option...
          value_out = cur_node_it->second.as<T>();
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
