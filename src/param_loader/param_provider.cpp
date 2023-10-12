#include <mrs_lib/param_provider.h>

namespace mrs_lib
{
  // Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
  // Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

  template bool ParamProvider::getParam<bool>(const std::string& name, bool& out_value) const;
  template bool ParamProvider::getParam<int>(const std::string& name, int& out_value) const;
  template bool ParamProvider::getParam<double>(const std::string& name, double& out_value) const;
  template bool ParamProvider::getParam<std::string>(const std::string& name, std::string& out_value) const;

  ParamProvider::ParamProvider(const ros::NodeHandle& nh, std::string node_name, const bool use_rosparam)
  : m_nh(nh), m_node_name(std::move(node_name)), m_use_rosparam(use_rosparam)
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
      ROS_ERROR_STREAM("[" << m_node_name << "]: Failed to parse file \"" << filepath << "\"! Parameters will not be loaded: " << e.what());
      return false;
    }
    catch (const YAML::BadFile& e)
    {
      ROS_ERROR_STREAM("[" << m_node_name << "]: File \"" << filepath << "\" does not exist! Parameters will not be loaded: " << e.what());
      return false;
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR_STREAM("[" << m_node_name << "]: YAML-CPP threw an exception! Parameters will not be loaded: " << e.what());
      return false;
    }
    return false;
  }

}
