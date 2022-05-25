// clang: MatousFormat
/**  \file
     \brief Defines DynamicReconfigureMgr - a convenience class for managing dynamic ROS parameters through dynamic reconfigure.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#ifndef DYNAMIC_RECONFIGURE_MGR_H
#define DYNAMIC_RECONFIGURE_MGR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <string>
#include <map>
#include <unordered_set>
#include <mutex>
#include <iostream>
#include <boost/any.hpp>
#include <Eigen/Dense>
#include <mrs_lib/param_loader.h>


namespace mrs_lib
{

/** DynamicReconfigureMgr CLASS //{ **/
// This class handles dynamic reconfiguration of parameters using dynamic_reconfigure server.
// Initialize this manager simply by instantiating an object of this templated class
// with the template parameter corresponding to the type of your config message, e.g. as
// DynamicReconfigureMgr<MyConfig> drmgr;
// This will automatically initialize the dynamic_reconfigure server and a callback method
// to asynchronously update the values in the config.
// Optionally, you can specify the ros NodeHandle to initialize the dynamic_reconfigure server
// and a flag 'print_values' to indicate whether to print new received values (only changed ones,
// default is true).
// The latest configuration is available through the public member 'config'. This should be
// changed externally with care since any change risks being overwritten in the next call to
// the 'dynamic_reconfigure_callback' method.
// Note that in case of a multithreaded ROS node, external mutexes _might_ be necessary
// to make access to the 'config' member thread-safe.
template <typename ConfigType>
class DynamicReconfigureMgr
{
  private:
    using callback_t = typename dynamic_reconfigure::Server<ConfigType>::CallbackType;
public:
  // this variable holds the latest received configuration
  ConfigType config;
  // initialize some stuff in the constructor
  DynamicReconfigureMgr(const ros::NodeHandle& nh = ros::NodeHandle("~"), bool print_values = true, std::string node_name = std::string(), const callback_t& user_callback = {})
      : m_not_initialized(true),
        m_loaded_invalid_default(false),
        m_print_values(print_values),
        m_node_name(node_name),
        m_server(m_server_mtx, nh),
        m_usr_cbf(user_callback),
        m_pl(nh, print_values, node_name)
  {
    // initialize the dynamic reconfigure callback
    m_server.setCallback(boost::bind(&DynamicReconfigureMgr<ConfigType>::dynamic_reconfigure_callback, this, _1, _2));
  };

  /* Constructor overloads //{ */
  // Convenience constructor to enable writing DynamicReconfigureMgr<MyConfig> drmgr(nh, node_name)
  DynamicReconfigureMgr(const ros::NodeHandle& nh, std::string node_name) : DynamicReconfigureMgr(nh, true, node_name){};
  // Convenience constructor to enable writing DynamicReconfigureMgr<MyConfig> drmgr(nh, "node_name")
  DynamicReconfigureMgr(const ros::NodeHandle& nh, const char* node_name) : DynamicReconfigureMgr(nh, std::string(node_name)){};
  // Convenience constructor to enable writing DynamicReconfigureMgr<MyConfig> drmgr(node_name)
  DynamicReconfigureMgr(std::string node_name) : DynamicReconfigureMgr(ros::NodeHandle("~"), node_name){};
  //}

  // pushes this config to the server
  void update_config(const ConfigType& cfg)
  {
    m_server.updateConfig(cfg);
  }

  // pushes the current config to the server
  void update_config()
  {
    m_server.updateConfig(config);
  }

  void publish_descriptions()
  {
    ConfigType dflt;
    m_server.getConfigDefault(dflt);
    m_server.setConfigDefault(dflt);
  }

  bool loaded_successfully()
  {
    return !m_not_initialized && !m_loaded_invalid_default && m_pl.loadedSuccessfully();
  }

private:
  bool m_not_initialized, m_loaded_invalid_default, m_print_values;
  std::string m_node_name;
  // dynamic_reconfigure server variables
  boost::recursive_mutex m_server_mtx;
  typename dynamic_reconfigure::Server<ConfigType> m_server;
  callback_t m_usr_cbf;

  ParamLoader m_pl;
  std::unordered_set<std::string> m_to_init;

  // the callback itself
  void dynamic_reconfigure_callback(ConfigType& new_config, uint32_t level)
  {
    if (m_print_values)
    {
      if (m_node_name.empty())
        ROS_INFO("Dynamic reconfigure request received");
      else
        ROS_INFO("[%s]: Dynamic reconfigure request received", m_node_name.c_str());
    }

    if (m_not_initialized)
    {
      load_defaults(new_config);
      update_config(new_config);
    }
    if (m_print_values)
    {
      print_changed_params(new_config);
    }
    m_not_initialized = false;
    config = new_config;
    if (m_usr_cbf)
      m_usr_cbf(new_config, level);
  }

  template <typename T>
  void load_param(const std::string& name, typename ConfigType::AbstractParamDescriptionConstPtr& descr, ConfigType& config)
  {
    using param_descr_t = typename ConfigType::template ParamDescription<T>;
    boost::shared_ptr<const param_descr_t> cast_descr = boost::dynamic_pointer_cast<const param_descr_t>(descr);
    m_pl.loadParam(name, config.*(cast_descr->field));
  }
  
  void load_defaults(ConfigType& new_config)
  {
    // Note that this part of the API is still unstable and may change! It was tested with ROS Kinetic and Melodic.
    std::vector<typename ConfigType::AbstractParamDescriptionConstPtr> descrs = new_config.__getParamDescriptions__();
    for (auto& descr : descrs)
    {
      std::string name = descr->name;
      size_t pos = name.find("__");
      while (pos != name.npos)
      {
        name.replace(pos, 2, "/");
        pos = name.find("__");
      }

      if (descr->type == "bool")
        load_param<bool>(name, descr, new_config);
      else if (descr->type == "int")
        load_param<int>(name, descr, new_config);
      else if (descr->type == "double")
        load_param<double>(name, descr, new_config);
      else if (descr->type == "str")
        load_param<std::string>(name, descr, new_config);
      else
      {
        ROS_ERROR("[%s]: Unknown parameter type: '%s'", m_node_name.c_str(), descr->type.c_str());
        m_loaded_invalid_default = true;
      }
    }
  }

  // method for printing names and values of new received parameters (prints only the changed ones) //{
  void print_changed_params(const ConfigType& new_config)
  {
    // Note that this part of the API is still unstable and may change! It was tested with ROS Kinetic and Melodic.
    std::vector<typename ConfigType::AbstractParamDescriptionConstPtr> descrs = new_config.__getParamDescriptions__();
    for (auto& descr : descrs)
    {
      boost::any val, old_val;
      descr->getValue(new_config, val);
      descr->getValue(config, old_val);
      std::string name = descr->name;
      const size_t pos = name.find("__");
      if (pos != name.npos)
      {
        if (m_not_initialized)
        {
          continue;
        } else
        {
          name.replace(pos, 2, "/");
        }
      }

      // try to guess the correct type of the parameter (these should be the only ones supported)
      int* intval;
      double* doubleval;
      bool* boolval;
      std::string* stringval;

      if (try_cast(val, intval))
      {
        if (m_not_initialized || !try_compare(old_val, intval))
          print_value(name, *intval);
      } else if (try_cast(val, doubleval))
      {
        if (m_not_initialized || !try_compare(old_val, doubleval))
          print_value(name, *doubleval);
      } else if (try_cast(val, boolval))
      {
        if (m_not_initialized || !try_compare(old_val, boolval))
          print_value(name, *boolval);
      } else if (try_cast(val, stringval))
      {
        if (m_not_initialized || !try_compare(old_val, stringval))
          print_value(name, *stringval);
      } else
      {
        print_value(name, std::string("unknown dynamic reconfigure type"));
      }
    }
  }
  //}
  
  // helper method for parameter printing
  template <typename T>
  inline void print_value(const std::string& name, const T& val)
  {
    if (m_node_name.empty())
      std::cout << "\t" << name << ":\t" << val << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << name << "':\t" << val);
  }
  // helper methods for automatic parameter value parsing
  template <typename T>
  inline bool try_cast(boost::any& val, T*& out)
  {
    return (out = boost::any_cast<T>(&val));
  };
  template <typename T>
  inline bool try_compare(boost::any& val, T*& to_what)
  {
    T* tmp;
    if ((tmp = boost::any_cast<T>(&val)))
    {
      /* std::cout << std::endl << *tmp << " vs " << *to_what << std::endl; */
      return *tmp == *to_what;
    } else
    {  // the value should not change during runtime - this should never happen (but its better to be safe than sorry)
      if (m_node_name.empty())
        ROS_WARN("DynamicReconfigure value type has changed - this should not happen!");
      else
        ROS_WARN_STREAM("[" << m_node_name << "]: DynamicReconfigure value type has changed - this should not happen!");
      return false;
    }
  };
};
//}

}

#endif // DYNAMIC_RECONFIGURE_MGR_H
