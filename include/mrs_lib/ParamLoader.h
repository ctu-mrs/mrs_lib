/**
 * This header defines several convenience functions for loading of ROS parameters
 * both static (e.g. from yaml files) and dynamic (using dynamic_reconfigure).
 * Please report any bugs to me (Matouš Vrba, vrbamato@fel.cvut.cz).
 * Hope you find these helpful :)
 **/
#ifndef PARAM_LOADER_H
#define PARAM_LOADER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <string>
#include <iostream>
#include <boost/any.hpp>
#include <Eigen/Dense>

namespace mrs_lib
{

/** ParamLoader CLASS //{ **/
class ParamLoader
{
private:
  bool m_load_successful, m_print_values;
  std::string m_node_name;
  const ros::NodeHandle& m_nh;

  void print_error(const std::string str)
  {
    if (m_node_name.empty())
      ROS_ERROR("%s", str.c_str());
    else
      ROS_ERROR("[%s]: %s", m_node_name.c_str(), str.c_str());
  }
  void print_warning(const std::string str)
  {
    if (m_node_name.empty())
      ROS_WARN("%s", str.c_str());
    else
      ROS_WARN("[%s]: %s", m_node_name.c_str(), str.c_str());
  }

  /* print_value function and overloads //{ */
  template <typename T>
  void print_value(const std::string& name, const T& value)
  {
    if (m_node_name.empty())
      std::cout << "\t" << name << ":\t" << value << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << name << "':\t" << value);
  };
  template <typename T>
  void print_value(const std::string& name, const std::vector<T>& value)
  {
    std::stringstream strstr;
    if (m_node_name.empty())
      strstr << "\t";
    strstr << name << ":\t ";
    for (size_t it = 0; it < value.size(); it++)
    {
      const T& elem = value.at(it);
      strstr << elem;
      if (it < value.size()-1)
        strstr  << ", ";
    }
    if (m_node_name.empty())
      std::cout << strstr.str() << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << strstr.str());
  };
   void print_value(const std::string& name, const Eigen::MatrixXd& value)
  {

    std::stringstream strstr;
    /* const Eigen::IOFormat fmt(4, 0, ", ", "\n", "\t\t[", "]"); */
    /* strstr << value.format(fmt); */
    const Eigen::IOFormat fmt;
    strstr << value.format(fmt);
    if (m_node_name.empty())
      std::cout << "\t" << name << ":\t" << std::endl << strstr.str() << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << name << "':" << std::endl << strstr.str());
  };
  //}

  // load_MatrixXd helper function for loading Eigen::MatrixXd matrices //{
  Eigen::MatrixXd load_MatrixXd(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols = -1, bool optional = true, bool swap = false)
  {
    Eigen::MatrixXd loaded = default_value;
    // this function only accepts dynamic columns (you can always transpose the matrix afterward)
    if (rows <= 0)
    {
      // if the parameter was compulsory, alert the user and set the flag
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + name);
      m_load_successful = false;
      return loaded;
    }
    
    bool check_size_exact = true;
    if (cols <= 0) // this means that the cols dimension is dynamic
      check_size_exact = false;

    std::vector<double> tmp_vec;
    // try to load the parameter
    bool success = m_nh.getParam(name, tmp_vec);
    // check if the loaded vector has correct length
    bool correct_size = (int)tmp_vec.size() == rows*cols;
    if (!check_size_exact)
      correct_size = (int)tmp_vec.size()%rows == 0; // if the cols dimension is dynamic, the size just has to be divisable by rows
    
    success = success && correct_size;
    if (success)
    {
      // if successfully loaded, everything is in order
      // transform the vector to the matrix
      if (cols <= 0)
        cols = tmp_vec.size()/rows;
      if (swap)
      {
        int tmp = cols;
        cols = rows;
        rows = tmp;
      }
      loaded = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>, Eigen::Unaligned>(tmp_vec.data(), rows, cols);
    } else
    {
      if (!correct_size)
      {
        // warn the user that this parameter was not successfully loaded because of wrong vector length (might be an oversight)
        print_warning(std::string("Matrix parameter ") + name + std::string(" could not be loaded because the vector has a wrong length!"));
      }
      // if it was not loaded, set the default value
      loaded = default_value;
      if (!optional)
      {
        // if the parameter was compulsory, alert the user and set the flag
        print_error(std::string("Could not load non-optional parameter ") + name);
        m_load_successful = false;
      }
    }
    // finally, return the resulting value
    return loaded;
  };
  //}

  /* load helper function //{ */
  // This function tries to load a parameter with name 'name' and a default value.
  // You can use the flag 'optional' to not throw a ROS_ERROR when the parameter
  // cannot be loaded and the flag 'print_values' to set whether the loaded
  // value and name of the parameter should be printed to cout.
  // If 'optional' is set to false and the parameter could not be loaded,
  // the flag 'load_successful' is set to false and a ROS_ERROR message
  // is printer.
  template <typename T>
  T load(const std::string& name, const T& default_value, bool optional = true)
  {
    T loaded = default_value;
    // try to load the parameter
    bool success = m_nh.getParam(name, loaded);
    if (success)
    {
      // if successfully loaded, everything is in order
      if (m_print_values)  // optionally, print its name and value
        print_value(name, loaded);
    } else
    {
      // if it was not loaded, set the default value
      loaded = default_value;
      if (!optional)
      {
        // if the parameter was compulsory, alert the user and set the flag
        print_error(std::string("Could not load non-optional parameter ") + name);
        m_load_successful = false;
      } else
      {
        // otherwise everything is fine and just print the name and value
        print_value(name, loaded);
      }
    }
    // finally, return the resulting value
    return loaded;
  };
  //}

public:
  // Default constructor
  ParamLoader(const ros::NodeHandle& nh, bool print_values = true, std::string node_name = std::string())
      : m_load_successful(true),
        m_print_values(print_values),
        m_node_name(node_name),
        m_nh(nh)
  {
            /* std::cout << "Initialized1 ParamLoader for node " << node_name << std::endl; */
  };
  /* Constructor overloads //{ */
  // Convenience overload to enable writing ParamLoader pr(nh, node_name);
  ParamLoader(const ros::NodeHandle& nh, std::string node_name)
      : ParamLoader(nh, true, node_name)
  {
            /* std::cout << "Initialized2 ParamLoader for node " << node_name << std::endl; */
  };
  // Convenience overload to enable writing ParamLoader pr(nh, "node_name");
  ParamLoader(const ros::NodeHandle& nh, const char* node_name)
      : ParamLoader(nh, std::string(node_name))
  {
            /* std::cout << "Initialized3 ParamLoader for node " << node_name << std::endl; */
  };
  //}


  // This function indicates whether all compulsory parameters were successfully loaded.
  // It returns false if any of them was not loaded.
  bool loaded_successfully()
  {
    return m_load_successful;
  };

  /* load_param function for optional parameters //{ */
  template <typename T>
  T load_param(const std::string& name, const T& default_value)
  {
    return load(name, default_value, true);
  };
  template <typename T>
  void load_param2(const std::string& name, T& out_value, const T& default_value)
  {
    out_value = load(name, default_value, true);
  };
  //}

  /* load_param function for compulsory parameters //{ */
  // This is just a convenience wrapper function which works like 'load_param' with 'optional' flag set to false.
  template <typename T>
  T load_param(const std::string& name)
  {
    return load(name, T(), false);
  };
  template <typename T>
  void load_param2(const std::string& name, T& out_value)
  {
    out_value = load(name, T(), false);
  };
  //}
  
  // load_param function overload for Eigen::MatrixXd type //{
  Eigen::MatrixXd load_param(const std::string& name, const Eigen::MatrixXd& default_value)
  {
    int rows = default_value.rows();
    int cols = default_value.cols();
    return load_matrix_static(name, default_value, rows, cols, true);
  }
  Eigen::MatrixXd load_param(const std::string& name)
  {
    print_error(std::string("You must specify matrix dimensions for parameter ") + name + std::string(" (use load_matrix_static?)"));
    m_load_successful = false;
    return Eigen::MatrixXd();
  }
  //}
  
  // load_matrix_static function for loading of Eigen::MatrixXd with known dimensions //{
  Eigen::MatrixXd load_matrix_static(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols, bool optional = true)
  {
    Eigen::MatrixXd loaded = default_value;
    // first, check that at least one dimension is set
    if (rows <= 0 || cols <= 0)
    {
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + name + std::string(" (use load_matrix_dynamic?)"));
      m_load_successful = false;
      return loaded;
    }

    loaded = load_MatrixXd(name, default_value, rows, cols, optional, false);
    if (m_print_values && m_load_successful)
      print_value(name, loaded);
    return loaded;
  }
  Eigen::MatrixXd load_matrix_static(const std::string& name, int rows, int cols)
  {
    return load_matrix_static(name, Eigen::MatrixXd(), rows, cols, false);
  }
  void load_matrix_static2(const std::string& name, Eigen::MatrixXd& mat, int rows, int cols)
  {
    mat = load_matrix_static(name, Eigen::MatrixXd(), rows, cols, false);
  }
  //}
  
  // load_matrix_dynamic function for half-dynamic loading of Eigen::MatrixXd //{
  Eigen::MatrixXd load_matrix_dynamic(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols, bool optional = true)
  {
    Eigen::MatrixXd loaded = default_value;
    // first, check that at least one dimension is set
    if (rows <= 0 && cols <= 0)
    {
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + name + std::string(" (at least one dimension must be specified)"));
      m_load_successful = false;
      return loaded;
    }

    bool swap = false;
    if (rows <= 0)
    {
      int tmp = rows;
      rows = cols;
      cols = tmp;
      swap = true;
    }
    loaded = load_MatrixXd(name, default_value, rows, cols, optional, swap);
    if (m_print_values && m_load_successful)
      print_value(name, loaded);
    return loaded;
  }
  Eigen::MatrixXd load_matrix_dynamic(const std::string& name, int rows, int cols)
  {
    return load_matrix_dynamic(name, Eigen::MatrixXd(), rows, cols, false);
  }
  void load_matrix_dynamic2(const std::string& name, Eigen::MatrixXd& mat ,  int rows, int cols)
  {
    mat = load_matrix_dynamic(name, Eigen::MatrixXd(), rows, cols, false);
  }
  //}
  
};
//}

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
      : m_not_initialized(true), m_print_values(print_values), m_node_name(node_name), m_server(nh)
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


private:
  bool m_not_initialized, m_print_values;
  std::string m_node_name;
  // dynamic_reconfigure server variables
  typename dynamic_reconfigure::Server<ConfigType> m_server;
  typename dynamic_reconfigure::Server<ConfigType>::CallbackType m_cbf;

  // the callback itself
  void dynamic_reconfigure_callback(ConfigType& new_config, uint32_t level)
  {
    if (m_node_name.empty())
      ROS_INFO("Dynamic reconfigure request received:");
    else
      ROS_INFO("[%s]: Dynamic reconfigure request received:", m_node_name.c_str());
    if (m_print_values)
    {
      print_changed_params(new_config);
    }
    m_not_initialized = false;
    config = new_config;
  };

  // method for printing names and values of new received parameters (prints only the changed ones)
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

}  // namespace mrs_lib

#endif  // PARAM_LOADER_H
