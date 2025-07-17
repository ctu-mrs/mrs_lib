// clang: MatousFormat
/**  \file
     \brief Defines ParamProvider - a convenience class for seamlessly loading parameters from YAML or ROS.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Afzal Ahmad
 */
#pragma once

#include <concepts>
#include <memory>
#include <optional>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

namespace mrs_lib
{

  /*!
   * \brief Helper overload for printing of std::vectors.
   *
   * \param os          The output stream to send the printed vector to.
   * \param var         The vector to be printed.
   * \return            A reference to the output stream.
   */
  template <typename T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& var);

  /*!
   * \brief Helper overload for printing of std::maps.
   *
   * \param os          The output stream to send the printed map to.
   * \param var         The map to be printed.
   * \return            A reference to the output stream.
   */
  template <typename Key, typename Value>
  std::ostream& operator<<(std::ostream& os, const std::map<Key, Value>& var);

  /*!
   * \brief Helper overload for printing of rclcpp::ParameterType.
   *
   * \param os          The output stream to send the printed map to.
   * \param var         The parameter type to be printed.
   * \return            A reference to the output stream.
   */
  std::ostream& operator<<(std::ostream& os, rclcpp::ParameterType& var);

  /*!
   * \brief Convenince function to get the corresponding rclcpp::ParamType from a C++ type.
   *
   * \return            the rclcpp::ParamType enum value corresponding to the C++ type specified by the template.
   */
  template <typename T>
  rclcpp::ParameterType to_param_type();

  /** \brief Convenience concept of a numeric value (i.e. either integral or floating point, and not bool). */
  template <typename T>
  concept numeric = (std::integral<T> || std::floating_point<T>) && !std::same_as<T, bool>;

/*** ParamProvider CLASS //{ **/

/**
 * \brief Helper class for ParamLoader and DynparamMgr.
 *
 * This class abstracts away loading of parameters from ROS parameter server and directly from
 * YAML files ("static" parameters). The user can specify a number of YAML files that will be
 * parsed and when a parameter value is requested, these are checked first before attempting
 * to load the parameter from the ROS server (which can be slow). The YAML files are searched
 * in FIFO order and when a matching name is found in a file, its value is returned.
 *
 */
class ParamProvider
{
public:
  /** \brief Helper struct to distinguish standard names from resolved names.
   *
   * This struct serves as a strong type different from std::string (used for non-resolved parameter
   * names) so that the user cannot accidentally call a method expecting the "raw" name with a resolved name
   * and vice-versa.
   *
   */
  struct resolved_name_t;

  /** \brief Helper struct for a numeric range with named members to make the code a bit more readable. */
  template <typename T>
  struct range_t
  {
    /** \brief Minimal value of a parameter. */
    T minimum;
    /** \brief Maximal value of a parameter. */
    T maximum;
  };

  /** \brief Struct of options when declaring a parameter to ROS.
   *
   * If the optionals are not filled (i.e. equal to std::nullopt), no default value, minimum
   * or maximum will be defined for the declared variable.
   *
   */
  template <typename T>
  struct declare_options_t
  {
    /** \brief If true, the parameter will be dynamically reconfigurable, otherwise it will be declared as read-only. */
    bool reconfigurable = false;
    /** \brief An optional default value to initialize the parameter with. */
    std::optional<T> default_value = std::nullopt;
    /** \brief An optional range of valid values of the parameter (only for numerical types). */
    std::optional<range_t<T>> range = std::nullopt;
  };

  /** \brief Struct of options when getting a parameter from ROS.
   *
   * If the optionals are not filled (i.e. equal to std::nullopt), the values set in the ParamProvider class are used.
   *
   */
  template <typename T>
  struct get_options_t
  {
    /** \brief Iff true, the parameter will be declared to ROS even if it's value was loaded from a YAML. */
    bool always_declare = false;
    /** \brief Iff false, loading from YAML will be skipped even if some YAML files were specified. */
    bool use_yaml = true;
    /** \brief Specifies whether the parameter should be attempted to be loaded from ROS if it cannot be loaded from a YAML. */
    std::optional<bool> use_rosparam = std::nullopt;
    /** \brief If filled, overrides any prefix set using the setPrefix() method. */
    std::optional<std::string> prefix = std::nullopt;
    /** \brief Options when declaring a parameter to ROS (see the declare_options_t<T> documentation). */
    declare_options_t<T> declare_options = {};
  };

  /** \brief Struct of options when setting a parameter to ROS.
   *
   * If the optionals are not filled (i.e. equal to std::nullopt), the values set in the ParamProvider class are used.
   *
   */
  template <typename T>
  struct set_options_t
  {
    /** \brief If filled, overrides any prefix set using the setPrefix() method. */
    std::optional<std::string> prefix = std::nullopt;
    /** \brief Options when declaring a parameter to ROS (see the declare_options_t<T> documentation). */
    declare_options_t<T> declare_options = {};
  };

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
   * \return                true iff the parameter was successfully loaded.
   */
  template <typename T>
  bool getParam(const std::string& param_name, T& value_out) const;

  /*!
   * \brief Gets the value of a parameter.
   *
   * Firstly, the parameter is attempted to be loaded from the YAML files added by the addYamlFile() method
   * in the same order that they were added. If the parameter is not found in any YAML file, and the use_rosparam
   * flag of the constructor is true, the ParamProvider will declare it in ROS and attempt to load it from ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param value_out       Output argument that will hold the value of the loaded parameter, if successfull. Not modified otherwise.
   * \param opts            Options regarding getting and declaring the parameter (see the get_options_t<T> documentation).
   * \return                true iff the parameter was successfully loaded.
   */
  template <typename T>
  bool getParam(const resolved_name_t& resolved_name, T& value_out, const get_options_t<T>& opts = {}) const;

  /*!
   * \brief Sets the value of a parameter to ROS.
   *
   * This method sets the parameter to ROS with the desired value.
   *
   * \param param_name      Name of the parameter to be set. Namespaces should be separated with a forward slash '/'.
   * \param value           The desired value of the parameter.
   * \return                true iff the parameter was successfully set.
   */
  template <typename T>
  bool setParam(const std::string& param_name, const T& value) const;

  /*!
   * \brief Sets the value of a parameter to ROS.
   *
   * This method sets the parameter to ROS with the behavior controlled by the options allowing to
   * set a default value, valid value range (for numerical types), declare the parameter as reconfigurable, etc.
   *
   * \param param_name      Name of the parameter to be set. Namespaces should be separated with a forward slash '/'.
   * \param value           The desired value of the parameter.
   * \param opts            Options regarding setting and declaring the parameter (see the set_options_t<T> documentation).
   * \return                true iff the parameter was successfully set.
   */
  template <typename T>
  bool setParam(const resolved_name_t& resolved_name, const T& value, const set_options_t<T>& opts = {}) const;

  /*!
   * \brief Defines a parameter.
   *
   * This method only declares the parameter in ROS.
   *
   * \param param_name      Name of the parameter to be loaded.
   * \return                true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParam(const std::string& param_name) const;

  /*!
   * \brief Defines a parameter with a default value.
   *
   * This method declares the parameter in ROS and sets the default value if there is no value in ROS.
   *
   * \param param_name      Name of the parameter to be loaded.
   * \param default_value   The default value to be set if there is no value of the parameter.
   * \return                true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParam(const std::string& param_name, const T& default_value) const;

  /*!
   * \brief Defines a parameter with various options.
   *
   * This method declares the parameter in ROS with the behavior controlled by the options allowing
   * to set a default value, valid value range (for numerical types), declare the parameter as reconfigurable, etc.
   *
   * \param param_name      Name of the parameter to be loaded.
   * \param opts            Options regarding declaring the parameter (see the declare_options_t<T> documentation).
   * \return                true iff the parameter was successfully declared.
   */
  template <typename T>
  bool declareParam(const resolved_name_t& resolved_name, const declare_options_t<T>& opts = {}) const;

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

  template <typename T>
  bool ranges_match(const rcl_interfaces::msg::ParameterDescriptor& descriptor, const range_t<T>& declare_range) const;

  template <typename T>
  bool loadFromYaml(const resolved_name_t& resolved_name, T& value_out, const get_options_t<T>& opts) const;

  template <typename T>
  bool loadFromROS(const resolved_name_t& resolved_name, T& value_out, const get_options_t<T>& opts) const;
};
//}

}  // namespace mrs_lib

#ifndef PARAM_PROVIDER_HPP
#include <mrs_lib/impl/param_provider.hpp>
#endif
