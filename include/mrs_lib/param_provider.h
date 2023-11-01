// clang: MatousFormat
/**  \file
     \brief Defines ParamProvider - a helper class for ParamLoader.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef PARAM_PROVIDER_H
#define PARAM_PROVIDER_H

#include <yaml-cpp/yaml.h>
#include <ros/node_handle.h>

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
  class ParamProvider
  {
    public:

  /*!
    * \brief Main constructor.
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param node_name     Optional node name used when printing the loaded values or loading errors.
    * \param use_rosparam  If true, parameters that weren't found in the YAML files will be attempted to be loaded from ROS.
    */
      ParamProvider(const ros::NodeHandle& nh, std::string node_name, const bool use_rosparam = true);

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
    * flag of the constructor is true, the ParamProvider will attempt to load it from the ROS parameter server.
    *
    * \param param_name   Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
    * \param value_out    Output argument that will hold the value of the loaded parameter, if successfull. Not modified otherwise.
    * \return             Returns true iff the parameter was successfully loaded.
    */
      template <typename T>
      bool getParam(const std::string& param_name, T& value_out) const;

  /*!
    * \brief Specialization of getParam() for the XmlRpcValue type.
    *
    * The XmlRpc::XmlRpcValue can be useful for manual parsing of more complex types.
    *
    * \warning XmlRpc::XmlRpcValue parameters cannot be loaded from a YAML file - only from ROS!
    *
    * \param param_name   Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
    * \param value_out    Output argument that will hold the value of the loaded parameter, if successfull. Not modified otherwise.
    * \return             Returns true iff the parameter was successfully loaded.
    */
      bool getParam(const std::string& param_name, XmlRpc::XmlRpcValue& value_out) const;

    private:

      std::vector<YAML::Node> m_yamls;
      ros::NodeHandle m_nh;
      std::string m_node_name;
      bool m_use_rosparam;

      template <typename T>
      bool getParamImpl(const std::string& param_name, T& value_out) const;

      std::optional<YAML::Node> findYamlNode(const std::string& param_name) const;
  };
//}
}

#include "mrs_lib/impl/param_provider.hpp"

#endif // PARAM_PROVIDER_H
