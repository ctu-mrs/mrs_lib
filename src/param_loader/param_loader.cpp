// clang: MatousFormat
#include <mrs_lib/param_loader.h>

namespace mrs_lib
{

  // Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
  // Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

  template bool ParamLoader::loadParam<bool>(const std::string& name, bool& out_value, const bool& default_value);
  template bool ParamLoader::loadParamReusable<bool>(const std::string& name, bool& out_value, const bool& default_value);

  template bool ParamLoader::loadParam<int>(const std::string& name, int& out_value, const int& default_value);
  template bool ParamLoader::loadParamReusable<int>(const std::string& name, int& out_value, const int& default_value);

  template bool ParamLoader::loadParam<double>(const std::string& name, double& out_value, const double& default_value);
  template bool ParamLoader::loadParamReusable<double>(const std::string& name, double& out_value, const double& default_value);

  template bool ParamLoader::loadParam<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);
  template bool ParamLoader::loadParamReusable<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);

  template <>
  rclcpp::Duration ParamLoader::loadParam2<rclcpp::Duration>(const std::string& name, const rclcpp::Duration& default_value)
  {
    const auto secs = loadParam2<double>(name, default_value.seconds());
    return rclcpp::Duration::from_seconds(secs);
  }

  template <>
  rclcpp::Duration ParamLoader::loadParam2<rclcpp::Duration>(const std::string& name)
  {
    const auto secs = loadParam2<double>(name);
    return rclcpp::Duration::from_seconds(secs);
  }

  /* loadParam specializations for rclcpp::Duration type //{ */

  bool ParamLoader::loadParam(const std::string& name, rclcpp::Duration& out, const rclcpp::Duration& default_value)
  {
    double     secs;
    const bool ret = loadParam<double>(name, secs, default_value.seconds());
    out            = rclcpp::Duration::from_seconds(secs);
    return ret;
  }

  bool ParamLoader::loadParam(const std::string& name, rclcpp::Duration& out)
  {
    double     secs;
    const bool ret = loadParam<double>(name, secs);
    out            = rclcpp::Duration::from_seconds(secs);
    return ret;
  }

  //}

  /* loadParam specializations for std_msgs::msg::ColorRGBA type //{ */

  bool ParamLoader::loadParam(const std::string& name, std_msgs::msg::ColorRGBA& out, const std_msgs::msg::ColorRGBA& default_value)
  {
    std_msgs::msg::ColorRGBA res;
    bool                     ret = true;
    ret                          = ret & loadParam(name + "/r", res.r, default_value.r);
    ret                          = ret & loadParam(name + "/g", res.g, default_value.g);
    ret                          = ret & loadParam(name + "/b", res.b, default_value.b);
    ret                          = ret & loadParam(name + "/a", res.a, default_value.a);
    if (ret)
      out = res;
    return ret;
  }

  std_msgs::msg::ColorRGBA ParamLoader::loadParam2(const std::string& name, const std_msgs::msg::ColorRGBA& default_value)
  {
    std_msgs::msg::ColorRGBA ret;
    loadParam(name, ret, default_value);
    return ret;
  }

  //}

  ParamLoader::ParamLoader(const std::shared_ptr<rclcpp::Node>& node, bool printValues, std::string_view node_name)
      : m_load_successful(true), m_print_values(printValues), m_node_name(node_name), m_node(node), m_pp(node)
  {}

  /* Constructor overloads //{ */
  /*!
   * \brief Convenience overload to enable writing ParamLoader pl(nh, node_name);
   *
   * \param nh            The parameters will be loaded from rosparam using this node handle.
   * \param node_name     Optional node name used when printing the loaded values or loading errors.
   */
  ParamLoader::ParamLoader(const std::shared_ptr<rclcpp::Node>& node, std::string node_name)
    : ParamLoader(node, true, node_name)
  {}

  //}

  void ParamLoader::setPrefix(const std::string& prefix)
  {
    m_pp.setPrefix(prefix);
  }

  std::string ParamLoader::getPrefix()
  {
    return m_pp.getPrefix();
  }

  bool ParamLoader::loadedSuccessfully() const
  {
    return m_load_successful;
  }

  void ParamLoader::resetLoadedSuccessfully()
  {
    m_load_successful = true;
  }

  void ParamLoader::resetUniques()
  {
    m_loaded_params.clear();
  }

  bool ParamLoader::addYamlFile(const std::string& filepath)
  {
    return m_pp.addYamlFile(filepath);
  }

  bool ParamLoader::addYamlFileFromParam(const std::string& param_name)
  {
    std::string filepath;
    if (!loadParam(param_name, filepath))
      return false;
    return m_pp.addYamlFile(filepath);
  }

  void ParamLoader::copyYamls(const ParamLoader& param_loader)
  {
    m_pp.copyYamls(param_loader.m_pp);
  }

  mrs_lib::ParamProvider& ParamLoader::getParamProvider()
  {
    return m_pp;
  }

  bool ParamLoader::check_duplicit_loading(const resolved_name_t& name)
  {
    if (m_loaded_params.count(name))
    {
      printError(std::string("Tried to load parameter \"") + name.str + std::string("\" twice"));
      m_load_successful = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  /* printError and printWarning functions //{*/
  void ParamLoader::printError(const std::string& str)
  {
    if (m_node_name.empty())
      RCLCPP_ERROR_STREAM(m_node->get_logger(), str);
    else
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node_name << "]: " << str);
  }
  void ParamLoader::printWarning(const std::string& str)
  {
    if (m_node_name.empty())
      RCLCPP_WARN_STREAM(m_node->get_logger(), str);
    else
      RCLCPP_WARN_STREAM(m_node->get_logger(), "[" << m_node_name << "]: " << str);
  }
  //}

}
