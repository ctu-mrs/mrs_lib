// clang: MatousFormat
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

  template <typename T>
  rclcpp::ParameterType to_param_type();

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
  /** \brief Helper struct to distinguish standard names from resolved names.
   *
   * This struct serves as a strong type different from std::string (used for non-resolved parameter
   * names) so that the user cannot accidentally call a method expecting the "raw" name with a resolved name.
   *
   */
  struct resolved_name_t : std::string
  {};

  /*!
   * \brief Main constructor.
   *
   * \param nh            The parameters will be loaded from rosparam using this node handle.
   * \param use_rosparam  If true, parameters that weren't found in the YAML files will be attempted to be loaded from ROS.
   */
  ParamProvider(const std::shared_ptr<rclcpp::Node>& node, const bool use_rosparam = true);

  /*!
   * \brief Add a YAML file to be parsed and used for loading parameters.
   *
   * The first file added will be the first file searched for a parameter when using getParam().
   *
   * \param filepath  Path to the YAML file.
   */
  bool addYamlFile(const std::string& filepath);

  /*!
   * \brief Copy parsed YAMLs from another ParamProvider.
   *
   * \param param_provider  The ParamProvider object to copy the YAMLs from.
   */
  void copyYamls(const ParamProvider& param_provider);

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
   * \return                true iff the parameter was successfully loaded.
   */
  template <typename T>
  bool getParam(const std::string& param_name, T& value_out, const bool reconfigurable = false) const;

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
   * \return                true iff the parameter was successfully loaded.
   */
  template <typename T>
  bool getParam(const resolved_name_t& resolved_name, T& value_out, const bool reconfigurable = false) const;

  /*!
   * \brief Sets the value of a parameter.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param value_out       Output argument that will hold the value of the loaded parameter, if successfull. Not modified otherwise.
   * \param reconfigurable  If true, the paramter will be declared as dynamically reconfigurable (unless it was already defined as read-only).
   * \return                true iff the parameter was successfully loaded.
   */
  template <typename T>
  bool setParam(const std::string& param_name, const T& value, const bool reconfigurable = false) const;

  /*!
   * \brief Defines a parameter.
   *
   * This method only declares the parameter in ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param reconfigurable  If true, the paramter will be declared as dynamically reconfigurable (unless it was already defined as read-only).
   * \return                true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParam(const std::string& param_name, const bool reconfigurable = false) const;

  /*!
   * \brief Defines a parameter.
   *
   * This method only declares the parameter in ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param reconfigurable  If true, the paramter will be declared as dynamically reconfigurable (unless it was already defined as read-only).
   * \return                true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParam(const resolved_name_t& resolved_name, const bool reconfigurable) const;

  /*!
   * \brief Defines a parameter with a default value.
   *
   * This method declares the parameter in ROS and sets the default value if there is no value in ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param default_value   The default value to be set if there is no value of the parameter.
   * \param reconfigurable  If true, the paramter will be declared as dynamically reconfigurable (unless it was already defined as read-only).
   * \return                true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParamDefault(const std::string& param_name, const T& default_value, const bool reconfigurable) const;

  /*!
   * \brief Sets a prefix that will be applied to parameter names before subnode namespaces.
   *
   * The prefix will be applied as-is, so if you need to separate it from the parameter name
   * e.g. using a forward slash '/', you must include it in the prefix.
   *
   * \param prefix      The prefix to be applied to all parameter names.
   */
  void setPrefix(const std::string& prefix);

  /*!
   * \brief Returns the current parameter name prefix.
   *
   * \return The current prefix to be applied to all parameter names.
   */
  std::string getPrefix() const;

  /*!
   * \brief Returns the parameter name with prefixes and subnode namespaces.
   *
   * \param param_name      Name of the parameter.
   * \return                name of the parameter with the appropriate prefixes and subnode namespaces applied.
   */
  resolved_name_t resolveName(const std::string& param_name) const;

private:
  std::vector<std::shared_ptr<YAML::Node>> m_yamls;
  std::shared_ptr<rclcpp::Node> m_node;
  std::string m_node_name;
  bool m_use_rosparam;
  std::string m_prefix;

  std::optional<YAML::Node> findYamlNode(const resolved_name_t& resolved_name) const;
};
//}

}  // namespace mrs_lib

#ifndef PARAM_PROVIDER_HPP
#include <mrs_lib/impl/param_provider.hpp>
#endif
