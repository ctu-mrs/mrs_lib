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
* Example usage:
* \include src/param_loader/example.cpp
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
  const ros::NodeHandle& m_nh;
  std::unordered_set<std::string> loaded_params;

  /* printing helper functions //{ */
  /* print_error and print_warning functions //{*/
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
  //}

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
  };

  template <typename T1, typename T2>
  void print_value(const std::string& name, const std::map<T1, T2>& value)
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

  std::string print_value_recursive(const std::string& name, XmlRpc::XmlRpcValue& value, unsigned depth = 0)
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
            strstr << print_value_recursive(name, value[it], depth+1);
          }
          break;
        }
      case XmlRpc::XmlRpcValue::TypeStruct:
        {
          int it = 0;
          for (auto& pair : value)
          {
            strstr << std::endl;
            strstr << print_value_recursive(pair.first, pair.second, depth+1);
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
  };

  void print_value(const std::string& name, XmlRpc::XmlRpcValue& value)
  {
    const std::string txt = print_value_recursive(name, value);
    if (m_node_name.empty())
      std::cout << txt << std::endl;
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: parameter '" << txt);
  };

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
      print_error(std::string("Tried to load parameter ") + name + std::string(" twice"));
      m_load_successful = false;
      return true;
    } else
    {
      return false;
    }
  }
  //}

  /* load_matrix_static_internal helper function for loading static Eigen matrices //{ */
  template <int rows, int cols>
  Eigen::Matrix<double, rows, cols> load_matrix_static_internal(const std::string& name, const Eigen::Matrix<double, rows, cols>& default_value, optional_t optional, unique_t unique)
  {
    Eigen::MatrixXd dynamic = load_MatrixXd(name, default_value, rows, cols, optional, unique, NO_SWAP);
    if (dynamic.rows() == rows || dynamic.cols() == cols)
      return dynamic;
    else
      return default_value;
  }
  //}

  /* helper functions for loading Eigen::MatrixXd matrices //{ */
  // load_MatrixXd helper function for loading Eigen::MatrixXd matrices //{
  Eigen::MatrixXd load_MatrixXd(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols = -1, optional_t optional = OPTIONAL, unique_t unique = UNIQUE, swap_t swap = NO_SWAP, bool print_values = true)
  {
    Eigen::MatrixXd loaded = default_value;
    // first, check if the user already tried to load this parameter
    if (unique && check_duplicit_loading(name))
      return loaded;

    // this function only accepts dynamic columns (you can always transpose the matrix afterward)
    if (rows < 0)
    {
      // if the parameter was compulsory, alert the user and set the flag
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name));
      m_load_successful = false;
      return loaded;
    }
    bool expect_zero_matrix = rows == 0;
    if (expect_zero_matrix)
    {
      if (cols > 0)
      {
        print_error(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name) + ". One dimension indicates zero matrix, but other expects non-zero.");
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
    bool success = m_nh.getParam(name, tmp_vec);
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
            std::string("Matrix parameter ") + name
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
        print_error(std::string("Could not load non-optional parameter ") + resolved(name));
        cur_load_successful = false;
      }
    }

    // check if load was a success
    if (cur_load_successful)
    {
      if (m_print_values && print_values)
        print_value(name, loaded);
      loaded_params.insert(name);
    } else
    {
      m_load_successful = false;
    }
    // finally, return the resulting value
    return loaded;
  };
  //}

  /* load_matrix_array_internal helper function for loading an array of EigenXd matrices with known dimensions //{ */
  std::vector<Eigen::MatrixXd> load_matrix_array_internal(const std::string& name, const std::vector<Eigen::MatrixXd>& default_value, optional_t optional, unique_t unique)
  {
    int rows;
    std::vector<int> cols;
    bool success = true;
    success = success && m_nh.getParam(name + "/rows", rows);
    success = success && m_nh.getParam(name + "/cols", cols);

    std::vector<Eigen::MatrixXd> loaded;
    loaded.reserve(cols.size());

    int total_cols = 0;
    /* check correctness of loaded parameters so far calculate the total dimension //{ */

    if (!success)
    {
      print_error(std::string("Failed to load ") + resolved(name) + std::string("/rows or ") + resolved(name) + std::string("/cols"));
      m_load_successful = false;
      return default_value;
    }
    if (rows < 0)
    {
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name) + std::string(" (rows and cols must be >= 0)"));
      m_load_successful = false;
      return default_value;
    }
    for (const auto& col : cols)
    {
      if (col < 0)
      {
        print_error(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name) + std::string(" (rows and cols must be >= 0)"));
        m_load_successful = false;
        return default_value;
      }
      total_cols += col;
    }
    
    //}

    const Eigen::MatrixXd loaded_matrix = load_MatrixXd(name + "/data", Eigen::MatrixXd(), rows, total_cols, optional, unique, NO_SWAP, false);
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
      print_value(name + "/matrix#" + std::to_string(it), cur_mat);
    }
    return loaded;
  }
  //}

  /* load_matrix_static_internal helper function for loading EigenXd matrices with known dimensions //{ */
  Eigen::MatrixXd load_matrix_static_internal(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols, optional_t optional, unique_t unique)
  {
    Eigen::MatrixXd loaded = default_value;
    // first, check that at least one dimension is set
    if (rows <= 0 || cols <= 0)
    {
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name) + std::string(" (use load_matrix_dynamic?)"));
      m_load_successful = false;
      return loaded;
    }

    return load_MatrixXd(name, default_value, rows, cols, optional, unique, NO_SWAP);
  }
  //}

  /* load_matrix_dynamic_internal helper function for loading Eigen matrices with one dynamic (unspecified) dimension //{ */
  Eigen::MatrixXd load_matrix_dynamic_internal(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols, optional_t optional, unique_t unique)
  {
    Eigen::MatrixXd loaded = default_value;

    // next, check that at least one dimension is set
    if (rows <= 0 && cols <= 0)
    {
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + resolved(name) + std::string(" (at least one dimension must be specified)"));
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
    return load_MatrixXd(name, default_value, rows, cols, optional, unique, swap);
  }
  //}
  //}

  /* load helper function for generic types //{ */
  // This function tries to load a parameter with name 'name' and a default value.
  // You can use the flag 'optional' to not throw a ROS_ERROR when the parameter
  // cannot be loaded and the flag 'print_values' to set whether the loaded
  // value and name of the parameter should be printed to cout.
  // If 'optional' is set to false and the parameter could not be loaded,
  // the flag 'load_successful' is set to false and a ROS_ERROR message
  // is printer.
  // If 'unique' flag is set to false then the parameter is not checked
  // for being loaded twice
  template <typename T>
  T load(const std::string& name, const T& default_value, optional_t optional = OPTIONAL, unique_t unique = UNIQUE)
  {
    T loaded = default_value;
    if (unique && check_duplicit_loading(name))
      return loaded;

    bool cur_load_successful = true;
    // try to load the parameter
    bool success = m_nh.getParam(name, loaded);
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
    return loaded;
  };
  //}

public:
  /*!
    * \brief Main constructor.
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param print_values  If true, the loaded values will be printed to stdout using std::cout or ROS_INFO if node_name is not empty.
    * \param node_name     Optional node name used when printing the loaded values or loading errors.
    */
  ParamLoader(const ros::NodeHandle& nh, bool print_values = true, std::string node_name = std::string())
      : m_load_successful(true),
        m_print_values(print_values),
        m_node_name(node_name),
        m_nh(nh){
            /* std::cout << "Initialized1 ParamLoader for node " << node_name << std::endl; */
        };
  /* Constructor overloads //{ */
  /*!
    * \brief Convenience overload to enable writing ParamLoader pl(nh, node_name);
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param node_name     Optional node name used when printing the loaded values or loading errors.
    */
  ParamLoader(const ros::NodeHandle& nh, std::string node_name)
      : ParamLoader(nh, true, node_name){
            /* std::cout << "Initialized2 ParamLoader for node " << node_name << std::endl; */
        };
  /*!
    * \brief Convenience overload to enable writing ParamLoader pl(nh, "node_name");
    *
    * \param nh            The parameters will be loaded from rosparam using this node handle.
    * \param node_name     Optional node name used when printing the loaded values or loading errors.
    */
  ParamLoader(const ros::NodeHandle& nh, const char* node_name)
      : ParamLoader(nh, std::string(node_name)){
            /* std::cout << "Initialized3 ParamLoader for node " << node_name << std::endl; */
        };
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
    */
  template <typename T>
  void load_param(const std::string& name, T& out_value, const T& default_value)
  {
    out_value = load_param2<T>(name, default_value);
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
    return load<T>(name, default_value, OPTIONAL, UNIQUE);
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
    return load<T>(name, default_value, OPTIONAL, REUSABLE);
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
    */
  template <typename T>
  void load_param(const std::string& name, T& out_value)
  {
    out_value = load_param2<T>(name);
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
    return load<T>(name, T(), COMPULSORY, UNIQUE);
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
    return load<T>(name, COMPULSORY, REUSABLE);
  };
  //}

  /* load_param specializations for ros::Duration type //{ */

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    */
  void load_param(const std::string& name, ros::Duration& ret)
  {
    ret = load_param2(name);
  }

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \return              The loaded parameter value.
    */
  ros::Duration load_param2(const std::string& name)
  {
    const double secs = load_param2<double>(name);
    const ros::Duration ret(secs);
    return ret;
  }

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    */
  void load_param(const std::string& name, ros::Duration& ret, const ros::Duration& default_value)
  {
    ret = load_param2(name, default_value);
  }

  /*!
    * \brief An overload for loading ros::Duration.
    *
    * The duration will be loaded as a \p double, representing a number of seconds, and then converted to ros::Duration.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    */
  ros::Duration load_param2(const std::string& name, const ros::Duration& default_value)
  {
    const double secs = load_param2<double>(name, default_value.toSec());
    const ros::Duration ret(secs);
    return ret;
  }
  
  //}

  /* load_param specializations and convenience functions for Eigen::MatrixXd type //{ */

  /*!
    * \brief An overload for loading Eigen matrices.
    *
    * For compulsory Eigen matrices, use load_matrix_static() or load_matrix_dynamic().
    * Matrix dimensions are deduced from the provided default value.
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    */
  void load_param(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value)
  {
    mat = load_param2(name, default_value);
  }

  /*!
    * \brief An overload for loading Eigen matrices.
    *
    * For compulsory Eigen matrices, use load_matrix_static() or load_matrix_dynamic().
    * Matrix dimensions are deduced from the provided default value.
    * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
    * the default value is used.
    * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
    *
    * \param name          Name of the parameter in the rosparam server.
    * \param default_value This value will be used if the parameter name is not found in the rosparam server.
    * \return              The loaded parameter value.
    */
  Eigen::MatrixXd load_param2(const std::string& name, const Eigen::MatrixXd& default_value)
  {
    int rows = default_value.rows();
    int cols = default_value.cols();
    Eigen::MatrixXd loaded;
    load_matrix_dynamic(name, loaded, default_value, rows, cols);
    return loaded;
  }
  
  //}

  // load_matrix_static function for loading of static Eigen::Matrices //{

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
  void load_matrix_static(const std::string& name, Eigen::Matrix<double, rows, cols>& mat)
  {
    mat = load_matrix_static2<rows, cols>(name);
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
  void load_matrix_static(const std::string& name, Eigen::Matrix<double, rows, cols> mat, const Eigen::Matrix<double, rows, cols>& default_value)
  {
    mat = load_matrix_static2(name, default_value);
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
  Eigen::Matrix<double, rows, cols> load_matrix_static2(const std::string& name)
  {
    return load_matrix_static_internal(name, Eigen::Matrix<double, rows, cols>(), COMPULSORY, UNIQUE);
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
  Eigen::MatrixXd load_matrix_static2(const std::string& name, const Eigen::Matrix<double, rows, cols>& default_value)
  {
    return load_matrix_static_internal(name, default_value, OPTIONAL, UNIQUE);
  }
  //}

  // load_matrix_static function for loading of Eigen::MatrixXd with known dimensions //{

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
  void load_matrix_static(const std::string& name, Eigen::MatrixXd& mat, int rows, int cols)
  {
    mat = load_matrix_static2(name, rows, cols);
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
  void load_matrix_static(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    mat = load_matrix_static2(name, default_value, rows, cols);
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
  Eigen::MatrixXd load_matrix_static2(const std::string& name, int rows, int cols)
  {
    return load_matrix_static_internal(name, Eigen::MatrixXd(), rows, cols, COMPULSORY, UNIQUE);
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
  Eigen::MatrixXd load_matrix_static2(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    return load_matrix_static_internal(name, default_value, rows, cols, OPTIONAL, UNIQUE);
  }
  //}

  // load_matrix_dynamic function for half-dynamic loading of Eigen::MatrixXd //{

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
  void load_matrix_dynamic(const std::string& name, Eigen::MatrixXd& mat, int rows, int cols)
  {
    mat = load_matrix_dynamic2(name, rows, cols);
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
  void load_matrix_dynamic(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    mat = load_matrix_dynamic2(name, default_value, rows, cols);
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
  Eigen::MatrixXd load_matrix_dynamic2(const std::string& name, int rows, int cols)
  {
    return load_matrix_dynamic_internal(name, Eigen::MatrixXd(), rows, cols, COMPULSORY, UNIQUE);
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
  Eigen::MatrixXd load_matrix_dynamic2(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    return load_matrix_dynamic_internal(name, default_value, rows, cols, OPTIONAL, UNIQUE);
  }

  //}

  // load_matrix_array function for loading of an array of Eigen::MatrixXd with known dimensions //{
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
  void load_matrix_array(const std::string& name, std::vector<Eigen::MatrixXd>& mat)
  {
    mat = load_matrix_array2(name);
  }

  /*!
    * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
    *
    * This overload of the load_matrix_array() method takes a default value for the parameter, which is used in case a \c rosparam with the specified name is not
    * found in the \c rosparam server, instead of causing an unsuccessful load. This makes specifying the parameter value in the \c rosparam server optional.
    *
    * \param name           Name of the parameter in the rosparam server.
    * \param mat            Reference to the variable to which the parameter value will be stored (such as a class member variable).
    * \param default_value  The default value to be used in case the parameter is not found on the \c rosparam server.
    *
    */
  void load_matrix_array(const std::string& name, std::vector<Eigen::MatrixXd>& mat, const std::vector<Eigen::MatrixXd>& default_value)
  {
    mat = load_matrix_array2(name, default_value);
  }

  /*!
    * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
    *
    * This method works in the same way as the load_matrix_array() method for optional parameters, except that the loaded
    * parameter is returned and not stored in the reference parameter.
    *
    * \param name           Name of the parameter in the rosparam server.
    * \param default_value  The default value to be used in case the parameter is not found on the \c rosparam server.
    * \returns              The loaded parameter or the default value.
    *
    */
  std::vector<Eigen::MatrixXd> load_matrix_array2(const std::string& name, const std::vector<Eigen::MatrixXd>& default_value)
  {
    return load_matrix_array_internal(name, default_value, OPTIONAL, UNIQUE);
  }

  /*!
    * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
    *
    * This method works in the same way as the load_matrix_array() method for compulsory parameters, except that the loaded
    * parameter is returned and not stored in the reference parameter.
    *
    * \param name           Name of the parameter in the rosparam server.
    * \returns              The loaded parameter or a default constructed object of the respective type.
    *
    */
  std::vector<Eigen::MatrixXd> load_matrix_array2(const std::string& name)
  {
    return load_matrix_array_internal(name, std::vector<Eigen::MatrixXd>(), COMPULSORY, UNIQUE);
  }
  //}

  //}

};
//}

}  // namespace mrs_lib

#endif  // PARAM_LOADER_H
