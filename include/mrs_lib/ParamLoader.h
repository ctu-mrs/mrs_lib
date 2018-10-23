// clang: MatousFormat
/**
 * This header defines a convenience class for loading of static ROS parameters
 * Please report any bugs to me (Matouš Vrba, vrbamato@fel.cvut.cz).
 * Hope you find these helpful :)
 **/
#ifndef PARAM_LOADER_H
#define PARAM_LOADER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <string>
#include <map>
#include <unordered_set>
#include <mutex>
#include <iostream>
#include <boost/any.hpp>
#include <Eigen/Dense>

namespace mrs_lib
{

/** ParamLoader CLASS //{ **/
class ParamLoader
{
public:
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
  //}
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

  /* helper functions for loading Eigen::MatrixXd matrices //{ */
  // load_MatrixXd helper function for loading Eigen::MatrixXd matrices //{
  Eigen::MatrixXd load_MatrixXd(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols = -1, optional_t optional = OPTIONAL, unique_t unique = UNIQUE, swap_t swap = NO_SWAP)
  {
    Eigen::MatrixXd loaded = default_value;
    // first, check if the user already tried to load this parameter
    if (unique && check_duplicit_loading(name))
      return loaded;

    // this function only accepts dynamic columns (you can always transpose the matrix afterward)
    if (rows <= 0)
    {
      // if the parameter was compulsory, alert the user and set the flag
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + name);
      m_load_successful = false;
      return loaded;
    }

    bool cur_load_successful = true;
    bool check_size_exact = true;
    if (cols <= 0)  // this means that the cols dimension is dynamic
      check_size_exact = false;

    std::vector<double> tmp_vec;
    // try to load the parameter
    bool success = m_nh.getParam(name, tmp_vec);
    // check if the loaded vector has correct length
    bool correct_size = (int)tmp_vec.size() == rows * cols;
    if (!check_size_exact)
      correct_size = (int)tmp_vec.size() % rows == 0;  // if the cols dimension is dynamic, the size just has to be divisable by rows

    if (success && correct_size)
    {
      // if successfully loaded, everything is in order
      // transform the vector to the matrix
      if (cols <= 0)
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
        print_error(std::string("Could not load non-optional parameter ") + name);
        cur_load_successful = false;
      }
    }

    // check if load was a success
    if (cur_load_successful)
    {
      if (m_print_values)
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

  /* load_matrix_static_internal helper function for loading static Eigen matrices //{ */
  Eigen::MatrixXd load_matrix_static_internal(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols, optional_t optional, unique_t unique)
  {
    Eigen::MatrixXd loaded = default_value;
    // first, check that at least one dimension is set
    if (rows <= 0 || cols <= 0)
    {
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + name + std::string(" (use load_matrix_dynamic?)"));
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
      print_error(std::string("Invalid expected matrix dimensions for parameter ") + name + std::string(" (at least one dimension must be specified)"));
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
        print_error(std::string("Could not load non-optional parameter ") + name);
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
  // Default constructor
  ParamLoader(const ros::NodeHandle& nh, bool print_values = true, std::string node_name = std::string())
      : m_load_successful(true),
        m_print_values(print_values),
        m_node_name(node_name),
        m_nh(nh){
            /* std::cout << "Initialized1 ParamLoader for node " << node_name << std::endl; */
        };
  /* Constructor overloads //{ */
  // Convenience overload to enable writing ParamLoader pr(nh, node_name);
  ParamLoader(const ros::NodeHandle& nh, std::string node_name)
      : ParamLoader(nh, true, node_name){
            /* std::cout << "Initialized2 ParamLoader for node " << node_name << std::endl; */
        };
  // Convenience overload to enable writing ParamLoader pr(nh, "node_name");
  ParamLoader(const ros::NodeHandle& nh, const char* node_name)
      : ParamLoader(nh, std::string(node_name)){
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
  void load_param(const std::string& name, T& out_value, const T& default_value)
  {
    out_value = load_param2<T>(name, default_value);
  };
  template <typename T>
  T load_param2(const std::string& name, const T& default_value)
  {
    return load<T>(name, default_value, OPTIONAL, UNIQUE);
  };
  template <typename T>
  T load_param_reusable(const std::string& name, const T& default_value)
  {
    return load<T>(name, default_value, OPTIONAL, REUSABLE);
  };
  //}

  /* load_param function for compulsory parameters //{ */
  // This is just a convenience wrapper function which works like 'load_param' with 'optional' flag set to false.
  template <typename T>
  void load_param(const std::string& name, T& out_value)
  {
    out_value = load_param2<T>(name);
  };
  template <typename T>
  T load_param2(const std::string& name)
  {
    return load<T>(name, T(), COMPULSORY, UNIQUE);
  };
  template <typename T>
  T load_param_reusable(const std::string& name)
  {
    return load<T>(name, COMPULSORY, REUSABLE);
  };
  //}

  /* load_param spetializations and convenience functions for Eigen::MatrixXd type //{ */
  // load_param function overload for Eigen::MatrixXd type //{
  void load_param(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value)
  {
    mat = load_param2(name, default_value);
  }
  void load_param(const std::string& name, Eigen::MatrixXd& mat)
  {
    mat = load_param2(name);
  }
  Eigen::MatrixXd load_param2(const std::string& name, const Eigen::MatrixXd& default_value)
  {
    int rows = default_value.rows();
    int cols = default_value.cols();
    Eigen::MatrixXd loaded;
    load_matrix_static(name, loaded, default_value, rows, cols);
    return loaded;
  }
  Eigen::MatrixXd load_param2(const std::string& name)
  {
    print_error(std::string("You must specify matrix dimensions for parameter ") + name + std::string(" (use load_matrix_static?)"));
    m_load_successful = false;
    return Eigen::MatrixXd();
  }
  //}

  // load_matrix_static function for loading of Eigen::MatrixXd with known dimensions //{
  void load_matrix_static(const std::string& name, Eigen::MatrixXd& mat, int rows, int cols)
  {
    mat = load_matrix_static2(name, rows, cols);
  }
  void load_matrix_static(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    mat = load_matrix_static2(name, default_value, rows, cols);
  }
  Eigen::MatrixXd load_matrix_static2(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    return load_matrix_static_internal(name, default_value, rows, cols, OPTIONAL, UNIQUE);
  }
  Eigen::MatrixXd load_matrix_static2(const std::string& name, int rows, int cols)
  {
    return load_matrix_static_internal(name, Eigen::MatrixXd(), rows, cols, COMPULSORY, UNIQUE);
  }
  //}

  // load_matrix_dynamic function for half-dynamic loading of Eigen::MatrixXd //{
  void load_matrix_dynamic(const std::string& name, Eigen::MatrixXd& mat, int rows, int cols)
  {
    mat = load_matrix_dynamic2(name, rows, cols);
  }
  void load_matrix_dynamic(const std::string& name, Eigen::MatrixXd& mat, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    mat = load_matrix_dynamic2(name, default_value, rows, cols);
  }
  Eigen::MatrixXd load_matrix_dynamic2(const std::string& name, const Eigen::MatrixXd& default_value, int rows, int cols)
  {
    return load_matrix_dynamic_internal(name, default_value, rows, cols, OPTIONAL, UNIQUE);
  }
  Eigen::MatrixXd load_matrix_dynamic2(const std::string& name, int rows, int cols)
  {
    return load_matrix_dynamic_internal(name, Eigen::MatrixXd(), rows, cols, COMPULSORY, UNIQUE);
  }
  //}
  //}
};
//}

}  // namespace mrs_lib

#endif  // PARAM_LOADER_H
