#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ros/node_handle.h>

namespace mrs_lib
{
  class ParamProvider
  {
    public:

      template <typename T>
      bool getParam(const std::string& param_name, T& value_out) const
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
            auto substr_start = std::cbegin(param_name);
            auto substr_end = substr_start;
            do
            {
              substr_end = std::find(substr_start, std::cend(param_name), delimiter);;
              const auto param_substr = param_name.substr(substr_start - std::begin(param_name), substr_end - std::begin(param_name));
              substr_start = substr_end+1;

              bool found = false;
              for (auto node_it = std::cbegin(cur_node_it->second); node_it != std::cend(cur_node_it->second); ++node_it)
              {
                if (node_it->first.as<std::string>() == param_substr)
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
            while (substr_end != std::end(param_name) && cur_node_it->second.IsMap());
          }

          if (loaded)
          {
            try
            {
              // try catch is the only option...
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

      ParamProvider(const ros::NodeHandle& nh, const std::string& node_name, const bool use_rosparam = true)
        : m_nh(nh), m_node_name(node_name), m_use_rosparam(use_rosparam)
      {
      }

      bool addYamlFile(const std::string& filepath)
      {
        try
        {
          const auto loaded_yaml = YAML::LoadFile(filepath);
          YAML::Node root;
          root["root"] = loaded_yaml;
          m_yamls.emplace_back(root);
          return true;
        }
        catch (const YAML::ParserException& e)
        {
          ROS_ERROR_STREAM("[" << m_node_name << "]: Failed to parse file \"" << filepath << "\"! Parameters will not be loaded.");
          return false;
        }
      }

    private:

      std::vector<YAML::Node> m_yamls;
      ros::NodeHandle m_nh;
      std::string m_node_name;
      bool m_use_rosparam;
  };
}
