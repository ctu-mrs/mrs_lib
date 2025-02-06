#include <memory>
#include <mrs_lib/param_provider.h>

namespace mrs_lib
{
  // Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
  // Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

  template bool ParamProvider::getParam<bool>(const std::string& name, bool& out_value) const;
  template bool ParamProvider::getParam<int>(const std::string& name, int& out_value) const;
  template bool ParamProvider::getParam<double>(const std::string& name, double& out_value) const;
  template bool ParamProvider::getParam<std::string>(const std::string& name, std::string& out_value) const;

  ParamProvider::ParamProvider(const std::shared_ptr<rclcpp::Node>& sub_node, std::string node_name, const bool use_rosparam)
  : m_node(sub_node), m_node_name(std::move(node_name)), m_use_rosparam(use_rosparam)
  {
  }

  bool ParamProvider::addYamlFile(const std::string& filepath)
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
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: Failed to parse file \"" << filepath << "\"! Parameters will not be loaded: " << e.what());
      return false;
    }
    catch (const YAML::BadFile& e)
    {
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: File \"" << filepath << "\" does not exist! Parameters will not be loaded: " << e.what());
      return false;
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: YAML-CPP threw an exception! Parameters will not be loaded: " << e.what());
      return false;
    }
    return false;
  }

  // bool ParamProvider::getParam(const std::string& param_name, XmlRpc::XmlRpcValue& value_out) const
  // {
  //   if (m_use_rosparam && m_node->get_parameter(param_name, value_out))
  //     return true;

  //   try
  //   {
  //     const auto found_node = findYamlNode(param_name);
  //     if (found_node.has_value())
  //       RCLCPP_WARN_STREAM(m_node->get_logger(), "[" << m_node_name << "]: Parameter \"" << param_name << "\" of desired type XmlRpc::XmlRpcValue is only available as a static parameter, which doesn't support loading of this type.");
  //   }
  //   catch (const YAML::Exception& e)
  //   {
  //     RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: YAML-CPP threw an unknown exception: " << e.what());
  //   }
  //   return false;
  // }

  std::optional<YAML::Node> ParamProvider::findYamlNode(const std::string& param_name) const
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
          substr_end = std::find(substr_start, std::cend(param_name), delimiter);
          // why can't substr or string_view take iterators? :'(
          const auto start_pos = std::distance(std::cbegin(param_name), substr_start);
          const auto count = std::distance(substr_start, substr_end);
          const std::string param_substr = param_name.substr(start_pos, count);
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
        return cur_node_it->second;
      }
    }

    return std::nullopt;
  }
}
