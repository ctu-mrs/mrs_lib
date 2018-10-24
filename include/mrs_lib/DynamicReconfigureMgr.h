// clang: MatousFormat
/**
 * This header defines a convenience class for loading of dynamic ROS parameters
 * Please report any bugs to me (Matouš Vrba, vrbamato@fel.cvut.cz).
 * Hope you find these helpful :)
 **/
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
#include <mrs_lib/ParamLoader.h>


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
public:
  // this variable holds the latest received configuration
  ConfigType config;
  // initialize some stuff in the constructor
  DynamicReconfigureMgr(const ros::NodeHandle& nh = ros::NodeHandle("~"), bool print_values = true, std::string node_name = std::string())
      : m_not_initialized(true),
       m_defaults_loaded(false),
       m_print_values(print_values),
       m_node_name(node_name),
       m_server(m_server_mtx, nh),
       m_pl(nh, print_values, node_name)
  {
    // initialize the dynamic reconfigure callback
    m_cbf = boost::bind(&DynamicReconfigureMgr<ConfigType>::dynamic_reconfigure_callback, this, _1, _2);
    m_server.setCallback(m_cbf);
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
  // pushes the actual config to the server
  void update_config()
  {
    m_server.updateConfig(config);
  }

  bool loaded_successfully()
  {
    return m_defaults_loaded;
  }


private:
  bool m_not_initialized, m_defaults_loaded, m_print_values;
  std::string m_node_name;
  // dynamic_reconfigure server variables
  boost::recursive_mutex m_server_mtx;
  typename dynamic_reconfigure::Server<ConfigType> m_server;
  typename dynamic_reconfigure::Server<ConfigType>::CallbackType m_cbf;

  ParamLoader m_pl;

  // the callback itself
  void dynamic_reconfigure_callback(ConfigType& new_config, [[maybe_unused]] uint32_t level)
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
      m_not_initialized = false;
    } else if (m_print_values)
    {
      print_changed_params(new_config);
    }
    config = new_config;
  };
  
  /* load_defaults() method //{ */
  void load_defaults(const ConfigType& new_config)
  {
    if (m_print_values)
    {
      if (m_node_name.empty())
        ROS_INFO("Loading default values of dynamically reconfigurable variables");
      else
        ROS_INFO("[%s]: Loading default values of dynamically reconfigurable variables", m_node_name.c_str());
    }
    // Note that this part of the API is still unstable and may change! It was tested with ROS Kinetic and Melodic.
    std::vector<typename ConfigType::AbstractParamDescriptionConstPtr> descrs = new_config.__getParamDescriptions__();
    for (auto& descr : descrs)
    {
      boost::any val, old_val;
      descr->getValue(new_config, val);
      descr->getValue(config, old_val);

      // try to guess the correct type of the parameter (these should be the only ones supported)
      int* intval;
      double* doubleval;
      bool* boolval;
      std::string* stringval;

      if (try_cast(val, intval))
      {
        m_pl.load_param(descr->name, *intval);
      } else if (try_cast(val, doubleval))
      {
        m_pl.load_param(descr->name, *doubleval);
      } else if (try_cast(val, boolval))
      {
        m_pl.load_param(descr->name, *boolval);
      } else if (try_cast(val, stringval))
      {
        m_pl.load_param(descr->name, *stringval);
      } else
      {
        print_value(descr->name, std::string("unknown dynamic reconfigure type"));
      }
    }

    if (m_pl.loaded_successfully())
    {
      if (m_print_values)
      {
        if (m_node_name.empty())
          ROS_INFO("Default values loaded");
        else
          ROS_INFO("[%s]: Default values loaded", m_node_name.c_str());
      }
      m_server.updateConfig(new_config);
      m_defaults_loaded = true;
    }
  }
  //}

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

      // try to guess the correct type of the parameter (these should be the only ones supported)
      int* intval;
      double* doubleval;
      bool* boolval;
      std::string* stringval;

      if (try_cast(val, intval))
      {
        if (!try_compare(old_val, intval) || m_not_initialized)
          print_value(descr->name, *intval);
      } else if (try_cast(val, doubleval))
      {
        if (!try_compare(old_val, doubleval) || m_not_initialized)
          print_value(descr->name, *doubleval);
      } else if (try_cast(val, boolval))
      {
        if (!try_compare(old_val, boolval) || m_not_initialized)
          print_value(descr->name, *boolval);
      } else if (try_cast(val, stringval))
      {
        if (!try_compare(old_val, stringval) || m_not_initialized)
          print_value(descr->name, *stringval);
      } else
      {
        print_value(descr->name, std::string("unknown dynamic reconfigure type"));
      }
    }
  }
  //}
  
  // helper method for parameter printing
  template <typename T>
  inline void print_value(std::string name, T val)
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
