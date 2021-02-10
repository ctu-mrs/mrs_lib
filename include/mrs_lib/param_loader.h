// clang: MatousFormat
/**  \file
     \brief Defines ParamLoader - a convenience class for loading static ROS parameters.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef PARAM_LOADER_H
#define PARAM_LOADER_H

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>
#include <unordered_set>
#include <iostream>

namespace mrs_lib
{

/*** ParamLoader CLASS //{ **/

/**
* \brief Convenience class for loading parameters from rosparam server.
*
* The parameters can be loaded as compulsory. If a compulsory parameter is not found
* on the rosparam server (e.g. because it is missing in the launchfile or the yaml config file),
* an internal flag is set to false, indicating that the parameter loading procedure failed.
* This flag can be checked using the loaded_successfully() method after all parameters were
* attempted to be loaded (see usage example usage below).
*
* The loaded parameter names and corresponding values are printed to stdout by default
* for user convenience. Special cases such as loading of Eigen matrices or loading
* of std::vectors of various values are also provided.
*
*/
class ParamLoader
{

private:
  enum unique_t
  {
    UNIQUE = true,
    REUSABLE = false
  };
  enum optional_t
  {
    OPTIONAL = true,
    COMPULSORY = false
  };
  enum swap_t
  {
    SWAP = true,
    NO_SWAP = false
  };

private:
  bool m_load_successful, m_print_values;
  rclcpp::Node& m_nh;
  rclcpp::Logger m_logger;
  std::unordered_set<std::string> loaded_params;

  /* printing helper functions //{ */
  /* printError and print_warning functions //{*/
  void print_error(const std::string str)
  {
    RCLCPP_ERROR(m_logger, "%s", str.c_str());
  }
  void print_warning(const std::string str)
  {
    RCLCPP_WARN(m_logger, "%s", str.c_str());
  }
  //}

  /* printValue function and overloads //{ */

  template <typename T>
  void print_value(const std::string& name, const T& value)
  {
    RCLCPP_INFO_STREAM(m_logger, "parameter '" << name << "':\t" << value);
  };

  template <typename T>
  void print_value(const std::string& name, const std::vector<T>& value)
  {
    std::stringstream strstr;
    strstr << name << ":\t";
    size_t it = 0;
    for (const auto& elem : value)
    {
      strstr << elem;
      if (it < value.size() - 1)
        strstr << ", ";
      it++;
    }
    RCLCPP_INFO_STREAM(m_logger, "parameter '" << strstr.str());
  };

  template <typename T1, typename T2>
  void print_value(const std::string& name, const std::map<T1, T2>& value)
  {
    std::stringstream strstr;
    strstr << name << ":" << std::endl;
    size_t it = 0;
    for (const auto& pair : value)
    {
      strstr << pair.first << " = " << pair.second;
      if (it < value.size() - 1)
        strstr << std::endl;
      it++;
    }
    RCLCPP_INFO_STREAM(m_logger, "parameter '" << strstr.str());
  };

  //}
  
  std::string resolved(const std::string& param_name)
  {
    return param_name;
  }
  //}

  /* check_duplicit_loading checks whether the parameter was already loaded - returns true if yes //{ */
  bool check_duplicit_loading(const std::string& name)
  {
    if (loaded_params.count(name))
    {
      print_error(std::string("Tried to load parameter ") + name + std::string(" twice"));
      m_load_successful = false;
      return true;
    } else
    {
      return false;
    }
  }
  //}

  /* load helper function for generic types //{ */
  // This function tries to load a parameter with name 'name' and a default value.
  // You can use the flag 'optional' to not throw a ROS_ERROR when the parameter
  // cannot be loaded and the flag 'printValues' to set whether the loaded
  // value and name of the parameter should be printed to cout.
  // If 'optional' is set to false and the parameter could not be loaded,
  // the flag 'load_successful' is set to false and a ROS_ERROR message
  // is printer.
  // If 'unique' flag is set to false then the parameter is not checked
  // for being loaded twice.
  // Returns a tuple, containing either the loaded or the default value and a bool,
  // indicating if the value was loaded (true) or the default value was used (false).
  template <typename T>
  std::pair<T, bool> load(const std::string& name, const T& default_value, optional_t optional = OPTIONAL, unique_t unique = UNIQUE)
  {
    T loaded = default_value;
    if (unique && check_duplicit_loading(name))
      return {loaded, false};

    bool cur_load_successful = true;
    // declare the parameter
    m_nh.declare_parameter(name);
    // try to load the parameter
    const bool success = m_nh.get_parameter(name, loaded);
    if (!success)
    {
      // if it was not loaded, set the default value
      loaded = default_value;
      if (!optional)
      {
        // if the parameter was compulsory, alert the user and set the flag
        print_error(std::string("Could not load non-optional parameter ") + resolved(name));
        cur_load_successful = false;
      }
    }

    if (cur_load_successful)
    {
      // everything is fine and just print the name and value if required
      if (m_print_values)
        print_value(name, loaded);
      // mark the param name as successfully loaded
      loaded_params.insert(name);
    } else
    {
      m_load_successful = false;
    }
    // finally, return the resulting value
    return {loaded, success};
  };
  //}

public:
  /*!
    * \brief Main constructor.
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param printValues   If true, the loaded values will be printed to stdout.
    */
  ParamLoader(rclcpp::Node& nh, bool printValues = true)
      : m_load_successful(true),
        m_print_values(printValues),
        m_nh(nh),
        m_logger(m_nh.get_logger())
        {};
  //}


  /*!
    * \brief Indicates whether all compulsory parameters were successfully loaded.
    *
    * \return false if any compulsory parameter was not loaded (is not present at rosparam server). Otherwise returns true.
    */
  bool loaded_successfully()
  {
    return m_load_successful;
  };

  /* load_param function for optional parameters //{ */
  /*!
    * \brief Loads a parameter from the rosparam server with a default value.
    *
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
    */
  template <typename T>
  bool load_param(const std::string& name, T& out_value, const T& default_value)
  {
    const auto [ret, success] = load<T>(name, default_value, OPTIONAL, UNIQUE);
    out_value = ret;
    return success;
  };
  /*!
    * \brief Loads a parameter from the rosparam server with a default value.
    *
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    */
  template <typename T>
  T load_param2(const std::string& name, const T& default_value)
  {
    const auto loaded = load<T>(name, default_value, OPTIONAL, UNIQUE);
    return loaded.first;
  };
  /*!
    * \brief Loads a parameter from the rosparam server with a default value.
    *
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can be loaded multiple times using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    */
  template <typename T>
  T load_param_reusable(const std::string& name, const T& default_value)
  {
    const auto loaded = load<T>(name, default_value, OPTIONAL, REUSABLE);
    return loaded.first;
  };
  //}

  /* load_param function for compulsory parameters //{ */
  /*!
    * \brief Loads a compulsory parameter from the rosparam server.
    *
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful (loaded_successfully() will return false).
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
    */
  template <typename T>
  bool load_param(const std::string& name, T& out_value)
  {
    const auto [ret, success] = load<T>(name, T(), COMPULSORY, UNIQUE);
    out_value = ret;
    return success;
  };
  /*!
    * \brief Loads a compulsory parameter from the rosparam server.
    *
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful (loaded_successfully() will return false).
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \return              The loaded parameter value.
    */
  template <typename T>
  T load_param2(const std::string& name)
  {
    const auto loaded = load<T>(name, T(), COMPULSORY, UNIQUE);
    return loaded.first;
  };

  /*!
    * \brief Loads a compulsory parameter from the rosparam server.
    *
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful (loaded_successfully() will return false).
    * Using this method, the parameter can be loaded multiple times using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \return              The loaded parameter value.
    */
  template <typename T>
  T load_param_reusable(const std::string& name)
  {
    const auto loaded = load<T>(name, T(), COMPULSORY, REUSABLE);
    return loaded.first;
  };
  //}

  //}

};
//}

}  // namespace mrs_lib

#endif  // PARAM_LOADER_H
