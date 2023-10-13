#ifndef PARAM_PROVIDER_H
#define PARAM_PROVIDER_H

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
      bool getParam(const std::string& param_name, T& value_out) const;

      bool getParam(const std::string& param_name, XmlRpc::XmlRpcValue& value_out) const;

      ParamProvider(const ros::NodeHandle& nh, std::string node_name, const bool use_rosparam = true);

      bool addYamlFile(const std::string& filepath);

    private:

      std::vector<YAML::Node> m_yamls;
      ros::NodeHandle m_nh;
      std::string m_node_name;
      bool m_use_rosparam;

      template <typename T>
      bool getParamImpl(const std::string& param_name, T& value_out) const;

      std::optional<YAML::Node> findYamlNode(const std::string& param_name) const;
  };

}

#include "mrs_lib/impl/param_provider.hpp"

#endif // PARAM_PROVIDER_H
