// clang: MatousFormat
/**  \file
     \brief Defines ParamLoader - a convenience class for loading static ROS parameters.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef PARAM_LOADER_H
#define PARAM_LOADER_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <unordered_set>
#include <iostream>
#include <Eigen/Dense>
#include <std_msgs/ColorRGBA.h>

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
* To load parameters into the `rosparam` server, use a launchfile prefferably.
* See documentation of ROS launchfiles here: http://wiki.ros.org/roslaunch/XML.
* Specifically, the `param` XML tag is used for loading parameters directly from the launchfile: http://wiki.ros.org/roslaunch/XML/param,
* and the `rosparam` XML tag tag is used for loading parameters from a `yaml` file: http://wiki.ros.org/roslaunch/XML/rosparam.
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
  std::string m_node_name;
  std::string m_prefix;
  const ros::NodeHandle& m_nh;
  std::unordered_set<std::string> loaded_params;

  /* printing helper functions //{ */
  /* printError and print_warning functions //{*/
  void printError(const std::string str)
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
  //}

  /* printValue function and overloads //{ */

  template <typename T>
  void printValue(const std::string& name, const T& value)
  {
    if (m_node_name.empty())
      std::cout << "\t" << name << ":\t" << value << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << name << "':\t" << value);
  }

  template <typename T>
  void printValue(const std::string& name, const std::vector<T>& value)
  {
    std::stringstream strstr;
    if (m_node_name.empty())
      strstr << "\t";
    strstr << name << ":\t";
    size_t it = 0;
    for (const auto& elem : value)
    {
      strstr << elem;
      if (it < value.size() - 1)
        strstr << ", ";
      it++;
    }
    if (m_node_name.empty())
      std::cout << strstr.str() << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << strstr.str());
  }

  template <typename T1, typename T2>
  void printValue(const std::string& name, const std::map<T1, T2>& value)
  {
    std::stringstream strstr;
    if (m_node_name.empty())
      strstr << "\t";
    strstr << name << ":" << std::endl;
    size_t it = 0;
    for (const auto& pair : value)
    {
      strstr << pair.first << " = " << pair.second;
      if (it < value.size() - 1)
        strstr << std::endl;
      it++;
    }
    if (m_node_name.empty())
      std::cout << strstr.str() << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << strstr.str());
  }

  void printValue(const std::string& name, const Eigen::MatrixXd& value)
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
  }

  std::string printValue_recursive(const std::string& name, XmlRpc::XmlRpcValue& value, unsigned depth = 0)
  {
    std::stringstream strstr;
    for (unsigned it = 0; it < depth; it++)
      strstr << "\t";
    strstr << name << ":";
    switch (value.getType())
    {
      case XmlRpc::XmlRpcValue::TypeArray:
        {
          for (int it = 0; it < value.size(); it++)
          {
            strstr << std::endl;
            const std::string name = "[" + std::to_string(it) + "]";
            strstr << printValue_recursive(name, value[it], depth+1);
          }
          break;
        }
      case XmlRpc::XmlRpcValue::TypeStruct:
        {
          int it = 0;
          for (auto& pair : value)
          {
            strstr << std::endl;
            strstr << printValue_recursive(pair.first, pair.second, depth+1);
            it++;
          }
          break;
        }
      default:
        {
          strstr << "\t" << value;
          break;
        }
    }
    return strstr.str();
  }

  void printValue(const std::string& name, XmlRpc::XmlRpcValue& value)
  {
    const std::string txt = printValue_recursive(name, value);
    if (m_node_name.empty())
      std::cout << txt << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << txt);
  }

  //}
  
  std::string resolved(const std::string& param_name)
  {
    return m_nh.resolveName(param_name);
  }
  //}

  /* check_duplicit_loading checks whether the parameter was already loaded - returns true if yes //{ */
  bool check_duplicit_loading(const std::string& name)
  {
    if (loaded_params.count(name))
    {
      printError(std::string("Tried to load parameter ") + name + std::string(" twice"));
      m_load_successful = false;
      return true;
    } else
    {
      return false;
    }
  }
  //}

  /* loadMatrixStatic_internal helper function for loading static Eigen matrices //{ */
  template <int rows, int cols>
  Eigen::Matrix<double, rows, cols> loadMatrixStatic_internal(const std::string& name, const Eigen::Matrix<double, rows, cols>& default_value, optional_t optional, unique_t unique)
  {
    Eigen::MatrixXd dynamic = loadMatrixXd(name, default_value, rows, cols, optional, unique, NO_SWAP);
    if (dynamic.rows() == rows || dynamic.cols() == cols)
      return dynamic;
    else
      return default_value;
  }
  //}

  /* helper functions for loading Eigen::MatrixXd matrices //{ */
  // loadMatrixXd helper function for loading Eigen::MatrixXd matrices //{
  Eigen::MatrixXd loadMatrixXd(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols = -1, optional_t optional = OPTIONAL, unique_t unique = UNIQUE, swap_t swap = NO_SWAP, bool printValues = true)
  {
    const std::string name_prefixed = m_prefix + name;
    Eigen::MatrixXd loaded = default_value;
    // first, check if the user already tried to load this parameter
    if (unique && check_duplicit_loading(name_prefixed))
      return loaded;

    // this function only accepts dynamic columns (you can always transpose the matrix afterward)
    if (rows < 0)
    {
      // if the parameter was compulsory, alert the user and set the flag
      printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name_prefixed));
      m_load_successful = false;
      return loaded;
    }
    bool expect_zero_matrix = rows == 0;
    if (expect_zero_matrix)
    {
      if (cols > 0)
      {
        printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name_prefixed) + ". One dimension indicates zero matrix, but other expects non-zero.");
        m_load_successful = false;
        return loaded;
      }
    }


    bool cur_load_successful = true;
    bool check_size_exact = true;
    if (cols <= 0)  // this means that the cols dimension is dynamic or a zero matrix is expected
      check_size_exact = false;

    std::vector<double> tmp_vec;
    // try to load the parameter
    bool success = m_nh.getParam(name_prefixed, tmp_vec);
    // check if the loaded vector has correct length
    bool correct_size = (int)tmp_vec.size() == rows * cols;
    if (!check_size_exact && !expect_zero_matrix)
      correct_size = (int)tmp_vec.size() % rows == 0;  // if the cols dimension is dynamic, the size just has to be divisable by rows

    if (success && correct_size)
    {
      // if successfully loaded, everything is in order
      // transform the vector to the matrix
      if (cols <= 0 && rows > 0)
        cols = tmp_vec.size() / rows;
      if (swap)
      {
        int tmp = cols;
        cols = rows;
        rows = tmp;
      }
      loaded = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>, Eigen::Unaligned>(tmp_vec.data(), rows, cols);
    } else
    {
      if (success && !correct_size)
      {
        // warn the user that this parameter was not successfully loaded because of wrong vector length (might be an oversight)
        std::string warning =
            std::string("Matrix parameter ") + name_prefixed
            + std::string(" could not be loaded because the vector has a wrong length " + std::to_string(tmp_vec.size()) + " instead of expected ");
        // process the message correctly based on whether the loaded matrix should be dynamic or static
        if (cols <= 0)  // for dynamic matrices
          warning = warning + std::string("number divisible by ") + std::to_string(rows);
        else  // for static matrices
          warning = warning + std::to_string(rows * cols);
        print_warning(warning);
      }
      // if it was not loaded, set the default value
      loaded = default_value;
      if (!optional)
      {
        // if the parameter was compulsory, alert the user and set the flag
        printError(std::string("Could not load non-optional parameter ") + resolved(name_prefixed));
        cur_load_successful = false;
      }
    }

    // check if load was a success
    if (cur_load_successful)
    {
      if (m_print_values && printValues)
        printValue(name_prefixed, loaded);
      loaded_params.insert(name_prefixed);
    } else
    {
      m_load_successful = false;
    }
    // finally, return the resulting value
    return loaded;
  }
  //}

  /* loadMatrixArray_internal helper function for loading an array of EigenXd matrices with known dimensions //{ */
  std::vector<Eigen::MatrixXd> loadMatrixArray_internal(const std::string& name, const std::vector<Eigen::MatrixXd>& default_value, optional_t optional, unique_t unique)
  {
    const std::string name_prefixed = m_prefix + name;
    int rows;
    std::vector<int> cols;
    bool success = true;
    success = success && m_nh.getParam(name_prefixed + "/rows", rows);
    success = success && m_nh.getParam(name_prefixed + "/cols", cols);

    std::vector<Eigen::MatrixXd> loaded;
    loaded.reserve(cols.size());

    int total_cols = 0;
    /* check correctness of loaded parameters so far calculate the total dimension //{ */

    if (!success)
    {
      printError(std::string("Failed to load ") + resolved(name_prefixed) + std::string("/rows or ") + resolved(name_prefixed) + std::string("/cols"));
      m_load_successful = false;
      return default_value;
    }
    if (rows < 0)
    {
      printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name_prefixed) + std::string(" (rows and cols must be >= 0)"));
      m_load_successful = false;
      return default_value;
    }
    for (const auto& col : cols)
    {
      if (col < 0)
      {
        printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name_prefixed) + std::string(" (rows and cols must be >= 0)"));
        m_load_successful = false;
        return default_value;
      }
      total_cols += col;
    }
    
    //}

    const Eigen::MatrixXd loaded_matrix = loadMatrixXd(name + "/data", Eigen::MatrixXd(), rows, total_cols, optional, unique, NO_SWAP, false);
    /* std::cout << "loaded_matrix: " << loaded_matrix << std::endl; */
    /* std::cout << "loaded_matrix: " << loaded_matrix.rows() << "x" << loaded_matrix.cols() << std::endl; */
    /* std::cout << "expected dims: " << rows << "x" << total_cols << std::endl; */
    if (loaded_matrix.rows() != rows || loaded_matrix.cols() != total_cols)
    {
      m_load_successful = false;
      return default_value;
    }

    int cols_loaded = 0;
    for (unsigned it = 0; it < cols.size(); it++)
    {
      const int cur_cols = cols.at(it);
      const Eigen::MatrixXd cur_mat = loaded_matrix.block(0, cols_loaded, rows, cur_cols);
      /* std::cout << "cur_mat: " << cur_mat << std::endl; */
      loaded.push_back(cur_mat);
      cols_loaded += cur_cols;
      printValue(name_prefixed + "/matrix#" + std::to_string(it), cur_mat);
    }
    return loaded;
  }
  //}

  /* loadMatrixStatic_internal helper function for loading EigenXd matrices with known dimensions //{ */
  Eigen::MatrixXd loadMatrixStatic_internal(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols, optional_t optional, unique_t unique)
  {
    Eigen::MatrixXd loaded = default_value;
    // first, check that at least one dimension is set
    if (rows <= 0 || cols <= 0)
    {
      printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name) + std::string(" (use loadMatrixDynamic?)"));
      m_load_successful = false;
      return loaded;
    }

    return loadMatrixXd(name, default_value, rows, cols, optional, unique, NO_SWAP);
  }
  //}

  /* loadMatrixDynamic_internal helper function for loading Eigen matrices with one dynamic (unspecified) dimension //{ */
  Eigen::MatrixXd loadMatrixDynamic_internal(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols, optional_t optional, unique_t unique)
  {
    Eigen::MatrixXd loaded = default_value;

    // next, check that at least one dimension is set
    if (rows <= 0 && cols <= 0)
    {
      printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name) + std::string(" (at least one dimension must be specified)"));
      m_load_successful = false;
      return loaded;
    }

    swap_t swap = NO_SWAP;
    if (rows <= 0)
    {
      int tmp = rows;
      rows = cols;
      cols = tmp;
      swap = SWAP;
    }
    return loadMatrixXd(name, default_value, rows, cols, optional, unique, swap);
  }
  //}
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
    const std::string name_prefixed = m_prefix + name;
    T loaded = default_value;
    if (unique && check_duplicit_loading(name_prefixed))
      return {loaded, false};

    bool cur_load_successful = true;
    // try to load the parameter
    const bool success = m_nh.getParam(name_prefixed, loaded);
    if (!success)
    {
      // if it was not loaded, set the default value
      loaded = default_value;
      if (!optional)
      {
        // if the parameter was compulsory, alert the user and set the flag
        printError(std::string("Could not load non-optional parameter ") + resolved(name_prefixed));
        cur_load_successful = false;
      }
    }

    if (cur_load_successful)
    {
      // everything is fine and just print the name_prefixed and value if required
      if (m_print_values)
        printValue(name_prefixed, loaded);
      // mark the param name_prefixed as successfully loaded
      loaded_params.insert(name_prefixed);
    } else
    {
      m_load_successful = false;
    }
    // finally, return the resulting value
    return {loaded, success};
  }
  //}

public:
  /*!
    * \brief Main constructor.
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param printValues  If true, the loaded values will be printed to stdout using std::cout or ROS_INFO if node_name is not empty.
    * \param node_name     Optional node name used when printing the loaded values or loading errors.
    */
  ParamLoader(const ros::NodeHandle& nh, bool printValues = true, std::string node_name = std::string())
      : m_load_successful(true),
        m_print_values(printValues),
        m_node_name(node_name),
        m_nh(nh)
  {
    /* std::cout << "Initialized1 ParamLoader for node " << node_name << std::endl; */
  }

  /* Constructor overloads //{ */
  /*!
    * \brief Convenience overload to enable writing ParamLoader pl(nh, node_name);
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param node_name     Optional node name used when printing the loaded values or loading errors.
    */
  ParamLoader(const ros::NodeHandle& nh, std::string node_name)
      : ParamLoader(nh, true, node_name)
  {
    /* std::cout << "Initialized2 ParamLoader for node " << node_name << std::endl; */
  }
  /*!
    * \brief Convenience overload to enable writing ParamLoader pl(nh, "node_name");
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param node_name     Optional node name used when printing the loaded values or loading errors.
    */
  ParamLoader(const ros::NodeHandle& nh, const char* node_name)
      : ParamLoader(nh, std::string(node_name))
  {
    /* std::cout << "Initialized3 ParamLoader for node " << node_name << std::endl; */
  }
  //}

  /* setPrefix function //{ */
  
  /*!
    * \brief All loaded parameters will be prefixed with this string.
    *
    * \param prefix  the prefix to be applied to all loaded parameters from now on.
    */
  void setPrefix(const std::string& prefix)
  {
    m_prefix = prefix;
  }
  
  //}

  /* getPrefix function //{ */
  
  /*!
    * \brief Returns the current parameter name prefix.
    *
    * \return the current prefix to be applied to the loaded parameters.
    */
  std::string getPrefix()
  {
    return m_prefix;
  }
  
  //}

  /* loadedSuccessfully function //{ */
  /*!
    * \brief Indicates whether all compulsory parameters were successfully loaded.
    *
    * \return false if any compulsory parameter was not loaded (is not present at rosparam server). Otherwise returns true.
    */
  bool loadedSuccessfully()
  {
    return m_load_successful;
  }
  //}

  /* loadParam function for optional parameters //{ */
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
  bool loadParam(const std::string& name, T& out_value, const T& default_value)
  {
    const auto [ret, success] = load<T>(name, default_value, OPTIONAL, UNIQUE);
    out_value = ret;
    return success;
  }
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
  T loadParam2(const std::string& name, const T& default_value)
  {
    const auto loaded = load<T>(name, default_value, OPTIONAL, UNIQUE);
    return loaded.first;
  }
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
  T loadParamReusable(const std::string& name, const T& default_value)
  {
    const auto loaded = load<T>(name, default_value, OPTIONAL, REUSABLE);
    return loaded.first;
  }
  //}

  /* loadParam function for compulsory parameters //{ */
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
  bool loadParam(const std::string& name, T& out_value)
  {
    const auto [ret, success] = load<T>(name, T(), COMPULSORY, UNIQUE);
    out_value = ret;
    return success;
  }
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
  T loadParam2(const std::string& name)
  {
    const auto loaded = load<T>(name, T(), COMPULSORY, UNIQUE);
    return loaded.first;
  }

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
  T loadParamReusable(const std::string& name)
  {
    const auto loaded = load<T>(name, T(), COMPULSORY, REUSABLE);
    return loaded.first;
  }
  //}

  /* loadParam specializations for ros::Duration type //{ */

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
    */
  bool loadParam(const std::string& name, ros::Duration& out, const ros::Duration& default_value)
  {
    double secs;
    const bool ret = loadParam<double>(name, secs, default_value.toSec());
    out = ros::Duration(secs);
    return ret;
  }

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
    */
  bool loadParam(const std::string& name, ros::Duration& out)
  {
    double secs;
    const bool ret = loadParam<double>(name, secs);
    out = ros::Duration(secs);
    return ret;
  }
  
  //}

  /* loadParam specializations for std_msgs::ColorRGBA type //{ */

  /*!
    * \brief An overload for loading std_msgs::ColorRGBA.
    *
    * The color will be loaded as several \p double -typed variables, representing the R, G, B and A color elements.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
    */
  bool loadParam(const std::string& name, std_msgs::ColorRGBA& out, const std_msgs::ColorRGBA& default_value = {})
  {
    std_msgs::ColorRGBA res;
    bool ret = true;
    ret = ret & loadParam(name+"/r", res.r, default_value.r);
    ret = ret & loadParam(name+"/g", res.g, default_value.g);
    ret = ret & loadParam(name+"/b", res.b, default_value.b);
    ret = ret & loadParam(name+"/a", res.a, default_value.a);
    if (ret)
      out = res;
    return ret;
  }

  /*!
    * \brief An overload for loading std_msgs::ColorRGBA.
    *
    * The color will be loaded as several \p double -typed variables, representing the R, G, B and A color elements.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    */
  std_msgs::ColorRGBA loadParam2(const std::string& name, const std_msgs::ColorRGBA& default_value = {})
  {
    std_msgs::ColorRGBA ret;
    loadParam(name, ret, default_value);
    return ret;
  }
  
  //}

  /* loadParam specializations and convenience functions for Eigen::MatrixXd type //{ */

  /*!
    * \brief An overload for loading Eigen matrices.
    *
    * For compulsory Eigen matrices, use loadMatrixStatic() or loadMatrixDynamic().
    * Matrix dimensions are deduced from the provided default value.
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    */
  void loadParam(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value)
  {
    mat = loadParam2(name, default_value);
  }

  /*!
    * \brief An overload for loading Eigen matrices.
    *
    * For compulsory Eigen matrices, use loadMatrixStatic() or loadMatrixDynamic().
    * Matrix dimensions are deduced from the provided default value.
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    */
  Eigen::MatrixXd loadParam2(const std::string& name, const Eigen::MatrixXd& default_value)
  {
    int rows = default_value.rows();
    int cols = default_value.cols();
    Eigen::MatrixXd loaded;
    loadMatrixDynamic(name, loaded, default_value, rows, cols);
    return loaded;
  }
  
  //}

  // loadMatrixStatic function for loading of static Eigen::Matrices //{

  /*!
    * \brief Specialized method for loading compulsory Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \tparam rows  Expected number of rows of the matrix.
    * \tparam cols  Expected number of columns of the matrix.
    *
    * \param name  Name of the parameter in the rosparam server.
    * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
    *
    */
  template <int rows, int cols>
  void loadMatrixStatic(const std::string& name, Eigen::Matrix<double, rows, cols>& mat)
  {
    mat = loadMatrixStatic2<rows, cols>(name);
  }

  /*!
    * \brief Specialized method for loading Eigen matrix parameters with default value.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \tparam rows          Expected number of rows of the matrix.
    * \tparam cols          Expected number of columns of the matrix.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param mat           Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    *
    */
  template <int rows, int cols>
  void loadMatrixStatic(const std::string& name, Eigen::Matrix<double, rows, cols> mat, const Eigen::Matrix<double, rows, cols>& default_value)
  {
    mat = loadMatrixStatic2(name, default_value);
  }

  /*!
    * \brief Specialized method for loading compulsory Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \tparam rows  Expected number of rows of the matrix.
    * \tparam cols  Expected number of columns of the matrix.
    *
    * \param name  Name of the parameter in the rosparam server.
    * \return      The loaded parameter value.
    *
    */
  template <int rows, int cols>
  Eigen::Matrix<double, rows, cols> loadMatrixStatic2(const std::string& name)
  {
    return loadMatrixStatic_internal<rows, cols>(name, Eigen::Matrix<double, rows, cols>::Zero(), COMPULSORY, UNIQUE);
  }

  /*!
    * \brief Specialized method for loading Eigen matrix parameters with default value.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \tparam rows          Expected number of rows of the matrix.
    * \tparam cols          Expected number of columns of the matrix.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    *
    */
  template <int rows, int cols>
  Eigen::MatrixXd loadMatrixStatic2(const std::string& name, const Eigen::Matrix<double, rows, cols>& default_value)
  {
    return loadMatrixStatic_internal<rows, cols>(name, default_value, OPTIONAL, UNIQUE);
  }
  //}

  // loadMatrixStatic function for loading of Eigen::MatrixXd with known dimensions //{

  /*!
    * \brief Specialized method for loading compulsory Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name  Name of the parameter in the rosparam server.
    * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param rows  Expected number of rows of the matrix.
    * \param cols  Expected number of columns of the matrix.
    */
  void loadMatrixStatic(const std::string& name, Eigen::MatrixXd& mat, int rows, int cols)
  {
    mat = loadMatrixStatic2(name, rows, cols);
  }

  /*!
    * \brief Specialized method for loading Eigen matrix parameters with default value.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param mat           Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \param rows          Expected number of rows of the matrix.
    * \param cols          Expected number of columns of the matrix.
    */
  void loadMatrixStatic(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    mat = loadMatrixStatic2(name, default_value, rows, cols);
  }

  /*!
    * \brief Specialized method for loading compulsory Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name  Name of the parameter in the rosparam server.
    * \param rows  Expected number of rows of the matrix.
    * \param cols  Expected number of columns of the matrix.
    * \return      The loaded parameter value.
    */
  Eigen::MatrixXd loadMatrixStatic2(const std::string& name, int rows, int cols)
  {
    return loadMatrixStatic_internal(name, Eigen::MatrixXd(), rows, cols, COMPULSORY, UNIQUE);
  }

  /*!
    * \brief Specialized method for loading Eigen matrix parameters with default value.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \param rows          Expected number of rows of the matrix.
    * \param cols          Expected number of columns of the matrix.
    * \return              The loaded parameter value.
    */
  Eigen::MatrixXd loadMatrixStatic2(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    return loadMatrixStatic_internal(name, default_value, rows, cols, OPTIONAL, UNIQUE);
  }
  //}

  // loadMatrixDynamic function for half-dynamic loading of Eigen::MatrixXd //{

  /*!
    * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name  Name of the parameter in the rosparam server.
    * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param rows  Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of columns and the size of the loaded array).
    * \param cols  Expected number of columns of the matrix (negative value indicates that the number of columns is to be deduced from the specified number of rows and the size of the loaded array).
    */
  void loadMatrixDynamic(const std::string& name, Eigen::MatrixXd& mat, int rows, int cols)
  {
    mat = loadMatrixDynamic2(name, rows, cols);
  }

  /*!
    * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \param mat           Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param rows          Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of columns and the size of the loaded array).
    * \param cols          Expected number of columns of the matrix (negative value indicates that the number of columns is to be deduced from the specified number of rows and the size of the loaded array).
    */
  void loadMatrixDynamic(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    mat = loadMatrixDynamic2(name, default_value, rows, cols);
  }

  /*!
    * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name  Name of the parameter in the rosparam server.
    * \param rows  Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of columns and the size of the loaded array).
    * \param cols  Expected number of columns of the matrix (negative value indicates that the number of columns is to be deduced from the specified number of rows and the size of the loaded array).
    * \return      The loaded parameter value.
    */
  Eigen::MatrixXd loadMatrixDynamic2(const std::string& name, int rows, int cols)
  {
    return loadMatrixDynamic_internal(name, Eigen::MatrixXd(), rows, cols, COMPULSORY, UNIQUE);
  }

  /*!
    * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
    *
    * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \param rows          Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of columns and the size of the loaded array).
    * \param cols          Expected number of columns of the matrix (negative value indicates that the number of columns is to be deduced from the specified number of rows and the size of the loaded array).
    * \return              The loaded parameter value.
    */
  Eigen::MatrixXd loadMatrixDynamic2(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    return loadMatrixDynamic_internal(name, default_value, rows, cols, OPTIONAL, UNIQUE);
  }

  //}

  // loadMatrixArray function for loading of an array of Eigen::MatrixXd with known dimensions //{
  /*!
    * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
    *
    * The number of rows and columns of the matrices to be loaded is specified in the \c rosparam parameter. Specifically, the \c name/rows value specifies the
    * number of rows, which must be common to all the loaded matrices (i.e. it is one integer >= 0), and the \c name/cols value specifies the number of columns of
    * each matrix (i.e. it is an array of integers > 0). The \c name/data array contains the values of the elements of the matrices and it must have length
    * \f$ r\sum_i c_i \f$, where \f$ r \f$ is the common number of rows and \f$ c_i \f$ is the number of columns of the \f$ i \f$-th matrix.
    * A typical structure of a \c yaml file, specifying the
    * matrix array to be loaded using this method, is
    *
    * \code{.yaml}
    *
    * matrix_array:
    *   rows: 3
    *   cols: [1, 2]
    *   data: [-5.0, -10.0, 23.0,
    *          -5.0, -10.0, 12.0,
    *           2.0,   4.0,  7.0]
    *
    * \endcode
    *
    * which will be loaded as a \c std::vector, containing one \f$ 3\times 1 \f$ matrix and one \f$ 3\times 2 \f$ matrix.
    *
    * If the dimensions of the loaded matrices do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully() will return false).
    * If the parameter with the specified name is not found on the \c rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the loading process is unsuccessful.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name  Name of the parameter in the rosparam server.
    * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
    *
    */
  void loadMatrixArray(const std::string& name, std::vector<Eigen::MatrixXd>& mat)
  {
    mat = loadMatrixArray2(name);
  }

  /*!
    * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
    *
    * This overload of the loadMatrixArray() method takes a default value for the parameter, which is used in case a \c rosparam with the specified name is not
    * found in the \c rosparam server, instead of causing an unsuccessful load. This makes specifying the parameter value in the \c rosparam server optional.
    *
    * \param name           Name of the parameter in the rosparam server.
    * \param mat            Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value  The default value to be used in case the parameter is not found on the \c rosparam server.
    *
    */
  void loadMatrixArray(const std::string& name, std::vector<Eigen::MatrixXd>& mat, const std::vector<Eigen::MatrixXd>& default_value)
  {
    mat = loadMatrixArray2(name, default_value);
  }

  /*!
    * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
    *
    * This method works in the same way as the loadMatrixArray() method for optional parameters, except that the loaded
    * parameter is returned and not stored in the reference parameter.
    *
    * \param name           Name of the parameter in the rosparam server.
    * \param default_value  The default value to be used in case the parameter is not found on the \c rosparam server.
    * \returns              The loaded parameter or the default value.
    *
    */
  std::vector<Eigen::MatrixXd> loadMatrixArray2(const std::string& name, const std::vector<Eigen::MatrixXd>& default_value)
  {
    return loadMatrixArray_internal(name, default_value, OPTIONAL, UNIQUE);
  }

  /*!
    * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
    *
    * This method works in the same way as the loadMatrixArray() method for compulsory parameters, except that the loaded
    * parameter is returned and not stored in the reference parameter.
    *
    * \param name           Name of the parameter in the rosparam server.
    * \returns              The loaded parameter or a default constructed object of the respective type.
    *
    */
  std::vector<Eigen::MatrixXd> loadMatrixArray2(const std::string& name)
  {
    return loadMatrixArray_internal(name, std::vector<Eigen::MatrixXd>(), COMPULSORY, UNIQUE);
  }
  //}

  //}

};
//}

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    */
  template <>
  ros::Duration ParamLoader::loadParam2<ros::Duration>(const std::string& name, const ros::Duration& default_value);

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \return              The loaded parameter value.
    */
  template <>
  ros::Duration ParamLoader::loadParam2<ros::Duration>(const std::string& name);

}  // namespace mrs_lib

#endif  // PARAM_LOADER_H
