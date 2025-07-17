// clang: MatousFormat
#include <memory>
#include <mrs_lib/param_provider.h>

namespace mrs_lib
{
  // Explicit instantiation of the templated functions to precompile them into mrs_lib and speed up compilation of user program.

  template bool ParamProvider::getParam<bool>(const std::string& name, bool& out_value) const;
  template bool ParamProvider::getParam<int>(const std::string& name, int& out_value) const;
  template bool ParamProvider::getParam<double>(const std::string& name, double& out_value) const;
  template bool ParamProvider::getParam<std::string>(const std::string& name, std::string& out_value) const;
  template bool ParamProvider::getParam<std::vector<bool>>(const std::string& name, std::vector<bool>& out_value) const;
  template bool ParamProvider::getParam<std::vector<uint8_t>>(const std::string& name, std::vector<uint8_t>& out_value) const;
  template bool ParamProvider::getParam<std::vector<int64_t>>(const std::string& name, std::vector<int64_t>& out_value) const;
  template bool ParamProvider::getParam<std::vector<double>>(const std::string& name, std::vector<double>& out_value) const;
  template bool ParamProvider::getParam<std::vector<std::string>>(const std::string& name, std::vector<std::string>& out_value) const;

  ParamProvider::ParamProvider(const std::shared_ptr<rclcpp::Node>& node, const bool use_rosparam)
  : m_node(node), m_node_name(m_node->get_name()), m_use_rosparam(use_rosparam)
  {
  }

  std::ostream& operator<<(std::ostream& os, rclcpp::ParameterType& var)
  {
    /* PARAMETER_NOT_SET */ 	
    /* PARAMETER_BOOL */ 	
    /* PARAMETER_INTEGER */ 	
    /* PARAMETER_DOUBLE */ 	
    /* PARAMETER_STRING */ 	
    /* PARAMETER_BYTE_ARRAY */ 	
    /* PARAMETER_BOOL_ARRAY */ 	
    /* PARAMETER_INTEGER_ARRAY */ 	
    /* PARAMETER_DOUBLE_ARRAY */ 	
    /* PARAMETER_STRING_ARRAY */ 
    switch (var)
    {
      case rclcpp::ParameterType::PARAMETER_NOT_SET: return os << "NOT_SET";
      case rclcpp::ParameterType::PARAMETER_BOOL: return os << "BOOL";
      case rclcpp::ParameterType::PARAMETER_INTEGER: return os << "PARAMETER_INTEGER";
      case rclcpp::ParameterType::PARAMETER_DOUBLE: return os << "PARAMETER_DOUBLE";
      case rclcpp::ParameterType::PARAMETER_STRING: return os << "PARAMETER_STRING";
      case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: return os << "PARAMETER_BYTE_ARRAY";
      case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: return os << "PARAMETER_BOOL_ARRAY";
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: return os << "PARAMETER_INTEGER_ARRAY";
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: return os << "PARAMETER_DOUBLE_ARRAY";
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: return os << "PARAMETER_STRING_ARRAY";
      default: return os << "INVALID";
    }
  }

  bool ParamProvider::addYamlFile(const std::string& filepath)
  {
    try
    {
      const auto loaded_yaml = YAML::LoadFile(filepath);
      auto root = std::make_shared<YAML::Node>();
      (*root)["root"] = loaded_yaml;
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

  void ParamProvider::copyYamls(const ParamProvider& param_provider)
  {
    m_yamls.insert(std::end(m_yamls), std::begin(param_provider.m_yamls), std::end(param_provider.m_yamls));
  }

  std::optional<YAML::Node> ParamProvider::findYamlNode(const resolved_name_t& resolved_name) const
  {
    for (const auto& yaml : m_yamls)
    {
      // Try to load the parameter sequentially as a map.
      auto cur_node_it = std::cbegin(*yaml);
      // The root should always be a pam
      if (!cur_node_it->second.IsMap())
        continue;

      bool loaded = true;
      {
        constexpr char delimiter = '/';
        auto substr_start = std::cbegin(resolved_name.str);
        auto substr_end = substr_start;
        do
        {
          substr_end = std::find(substr_start, std::cend(resolved_name.str), delimiter);
          // why can't substr or string_view take iterators? :'(
          const auto start_pos = std::distance(std::cbegin(resolved_name.str), substr_start);
          const auto count = std::distance(substr_start, substr_end);
          const std::string param_substr = resolved_name.str.substr(start_pos, count);
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
        while (substr_end != std::end(resolved_name.str) && cur_node_it->second.IsMap());
      }

      if (loaded)
      {
        return cur_node_it->second;
      }
    }

    return std::nullopt;
  }

  void ParamProvider::setPrefix(const std::string& prefix)
  {
    m_prefix = prefix;
  }

  std::string ParamProvider::getPrefix() const
  {
    return m_prefix;
  }

  ParamProvider::resolved_name_t ParamProvider::resolveName(const std::string& param_name) const
  {
    /* auto resolved_name = std::string(m_node->get_sub_namespace()) + "/" + param_name; */
    /* // reformat all '/' to '.' which is the new ROS2 delimeter */
    /* std::replace(resolved_name.begin() + 1, resolved_name.end(), '/', '.'); */
    /* return resolved_name; */
    /* const std::string node_name = m_node->get_fully_qualified_name(); */
    const std::string sub_namespace = m_node->get_sub_namespace();
    std::string resolved_name;
    if (sub_namespace.empty())
      resolved_name = param_name;
    else if (sub_namespace == "/")
    {
      resolved_name = sub_namespace + param_name;
    }
    else
    {
      resolved_name = sub_namespace + "/" + param_name;
      // reformat all '/' to '.' which is the new ROS2 delimeter
      /* std::replace(resolved_name.begin()+1, resolved_name.end(), '/', '.'); */
    }
    return resolved_name_t(m_prefix + resolved_name);
  }
} // namespace mrs_lib
