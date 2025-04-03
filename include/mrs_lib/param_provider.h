/**  \file
     \brief Defines ParamProvider - a convenience class for seamlessly loading parameters from YAML or ROS.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Afzal Ahmad
 */
#pragma once

#include <memory>
#include <optional>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace mrs_lib
{

/*** ParamProvider CLASS //{ **/

/**
 * \brief Helper class for ParamLoader.
 *
 * This class abstracts away loading of parameters from ROS parameter server and directly from
 * YAML files ("static" parameters). The user can specify a number of YAML files that will be
 * parsed and when a parameter value is requested, these are checked first before attempting
 * to load the parameter from the ROS server (which can be slow). The YAML files are searched
 * in FIFO order and when a matching name is found in a file, its value is returned.
 *
 */
class ParamProvider {
public:
  /*!
   * \brief Main constructor.
   *
   * \param nh            The parameters will be loaded from rosparam using this node handle.
   * \param node_name     Optional node name used when printing the loaded values or loading errors.
   * \param use_rosparam  If true, parameters that weren't found in the YAML files will be attempted to be loaded from ROS.
   */
  ParamProvider(const std::shared_ptr<rclcpp::Node>& node, std::string node_name, const bool use_rosparam = true);

  /*!
   * \brief Add a YAML file to be parsed and used for loading parameters.
   *
   * The first file added will be the first file searched for a parameter when using getParam().
   *
   * \param filepath  Path to the YAML file.
   */
  bool addYamlFile(const std::string& filepath);

  /*!
   * \brief Gets the value of a parameter.
   *
   * Firstly, the parameter is attempted to be loaded from the YAML files added by the addYamlFile() method
   * in the same order that they were added. If the parameter is not found in any YAML file, and the use_rosparam
   * flag of the constructor is true, the ParamProvider will declare it in ROS and attempt to load it from ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param value_out       Output argument that will hold the value of the loaded parameter, if successfull. Not modified otherwise.
   * \param reconfigurable  If true, the paramter will be declared as dynamically reconfigurable (unless it was already defined as read-only).
   * \return                Returns true iff the parameter was successfully loaded.
   */
  template <typename T>
  bool getParam(const std::string& param_name, T& value_out, const bool reconfigurable = false) const;

  /*!
   * \brief Defines a parameter.
   *
   * This method only declares the parameter in ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param reconfigurable  If true, the paramter will be declared as dynamically reconfigurable (unless it was already defined as read-only).
   * \return                Returns true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParam(const std::string& param_name, const bool reconfigurable = false) const;

  /*!
   * \brief Defines a parameter with a default value.
   *
   * This method declares the parameter in ROS and sets the default value if there is no value in ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param default_value   The default value to be set if there is no value of the parameter.
   * \param reconfigurable  If true, the paramter will be declared as dynamically reconfigurable (unless it was already defined as read-only).
   * \return                Returns true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParamDefault(const std::string& param_name, const T& default_value, const bool reconfigurable) const;

private:
  std::vector<YAML::Node>       m_yamls;
  std::shared_ptr<rclcpp::Node> m_node;
  std::string                   m_node_name;
  bool                          m_use_rosparam;

  std::string resolveName(const std::string& param_name) const;
  std::optional<YAML::Node> findYamlNode(const std::string& param_name) const;
};
//}

}  // namespace mrs_lib

#ifndef PARAM_PROVIDER_HPP
#include <mrs_lib/impl/param_provider.hpp>
#endif
