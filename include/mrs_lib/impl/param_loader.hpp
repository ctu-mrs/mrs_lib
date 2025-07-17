// clang: MatousFormat
#pragma once

#include <mrs_lib/param_loader.h>

namespace mrs_lib
{

  /* printValue function and overloads //{ */

  template <typename T>
  void ParamLoader::printValue(const resolved_name_t& name, const T& value) const
  {
    if (m_node_name.empty())
      std::cout << "\t" << name << ":\t" << value << std::endl;
    else
      RCLCPP_INFO_STREAM(m_node->get_logger(), "[" << m_node_name << "]: parameter '" << name << "':\t" << value);
  }

  template <typename T>
  std::ostream& operator<<(std::ostream& os, const Eigen::MatrixX<T>& var)
  {
    /* const Eigen::IOFormat fmt(4, 0, ", ", "\n", "\t\t[", "]"); */
    /* strstr << value.format(fmt); */
    const Eigen::IOFormat fmt;
    return os << var.format(fmt);
  }
  //}

  /* loading helper functions //{ */

  /* helper functions for loading dynamic Eigen matrices //{ */
  // loadMatrixX helper function for loading dynamic Eigen matrices //{
  template <typename T>
  std::pair<ParamLoader::MatrixX<T>, bool> ParamLoader::loadMatrixX(const std::string& name, const MatrixX<T>& default_value, int rows, int cols, optional_t optional, unique_t unique, swap_t swap, bool printValues)
  {
    const auto resolved_name = m_pp.resolveName(name);
    MatrixX<T> loaded = default_value;
    bool used_rosparam_value = false;
    // first, check if the user already tried to load this parameter
    if (unique && check_duplicit_loading(resolved_name))
      return {loaded, used_rosparam_value};

    // this function only accepts dynamic columns (you can always transpose the matrix afterward)
    if (rows < 0) {
      // if the parameter was compulsory, alert the user and set the flag
      printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved_name.str);
      m_load_successful = false;
      return {loaded, used_rosparam_value};
    }
    const bool expect_zero_matrix = rows == 0;
    if (expect_zero_matrix) {
      if (cols > 0) {
        printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved_name.str +
                   ". One dimension indicates zero matrix, but other expects non-zero.");
        m_load_successful = false;
        return {loaded, used_rosparam_value};
      }
    }

    bool cur_load_successful = true;
    bool check_size_exact    = true;
    if (cols <= 0)  // this means that the cols dimension is dynamic or a zero matrix is expected
      check_size_exact = false;

    std::vector<T> tmp_vec;
    // try to load the parameter
    const bool success = m_pp.getParam(resolved_name, tmp_vec);
    // check if the loaded vector has correct length
    bool correct_size = (int)tmp_vec.size() == rows * cols;
    if (!check_size_exact && !expect_zero_matrix)
      correct_size = (int)tmp_vec.size() % rows == 0;  // if the cols dimension is dynamic, the size just has to be divisable by rows

    if (success && correct_size) {
      // if successfully loaded, everything is in order
      // transform the vector to the matrix
      if (cols <= 0 && rows > 0)
        cols = tmp_vec.size() / rows;
      if (swap)
        std::swap(rows, cols);
      loaded              = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, Eigen::Unaligned>(tmp_vec.data(), rows, cols);
      used_rosparam_value = true;
    } else {
      if (success && !correct_size) {
        // warn the user that this parameter was not successfully loaded because of wrong vector length (might be an oversight)
        std::string warning =
            std::string("Matrix parameter ") + resolved_name.str +
            std::string(" could not be loaded because the vector has a wrong length " + std::to_string(tmp_vec.size()) + " instead of expected ");
        // process the message correctly based on whether the loaded matrix should be dynamic or static
        if (cols <= 0)  // for dynamic matrices
          warning = warning + std::string("number divisible by ") + std::to_string(rows);
        else  // for static matrices
          warning = warning + std::to_string(rows * cols);
        printWarning(warning);
      }
      // if it was not loaded, the default value is used (set at the beginning of the function)
      if (!optional) {
        // if the parameter was compulsory, alert the user and set the flag
        printError(std::string("Could not load non-optional parameter ") + resolved_name.str);
        cur_load_successful = false;
      }
    }

    // check if load was a success
    if (cur_load_successful) {
      if (m_print_values && printValues)
        printValue(resolved_name, loaded);
      m_loaded_params.insert(resolved_name);
    } else {
      m_load_successful = false;
    }
    // finally, return the resulting value
    return {loaded, used_rosparam_value};
  }
  //}

  /* loadMatrixStatic_internal helper function for loading static Eigen matrices //{ */
  template <int rows, int cols, typename T>
  std::pair<Eigen::Matrix<T, rows, cols>, bool> ParamLoader::loadMatrixStatic_internal(const std::string& name, const Eigen::Matrix<T, rows, cols>& default_value, optional_t optional, unique_t unique)
  {
    const auto [dynamic, loaded_ok] = loadMatrixX(name, MatrixX<T>(default_value), rows, cols, optional, unique, NO_SWAP);
    return {dynamic, loaded_ok};
  }
  //}

  /* loadMatrixKnown_internal helper function for loading EigenXd matrices with known dimensions //{ */
  template <typename T>
  std::pair<ParamLoader::MatrixX<T>, bool> ParamLoader::loadMatrixKnown_internal(const std::string& name, const MatrixX<T>& default_value, int rows, int cols, optional_t optional, unique_t unique)
  {
    MatrixX<T> loaded = default_value;
    // first, check that at least one dimension is set
    if (rows <= 0 || cols <= 0) {
      printError(std::string("Invalid expected matrix dimensions for parameter ") + m_pp.resolveName(name).str + std::string(" (use loadMatrixDynamic?)"));
      m_load_successful = false;
      return {loaded, false};
    }

    return loadMatrixX(name, default_value, rows, cols, optional, unique, NO_SWAP);
  }
  //}

  /* loadMatrixDynamic_internal helper function for loading Eigen matrices with one dynamic (unspecified) dimension //{ */
  template <typename T>
  std::pair<ParamLoader::MatrixX<T>, bool> ParamLoader::loadMatrixDynamic_internal(const std::string& name, const MatrixX<T>& default_value, int rows, int cols, optional_t optional, unique_t unique)
  {
    MatrixX<T> loaded = default_value;

    // next, check that at least one dimension is set
    if (rows <= 0 && cols <= 0) {
      printError(std::string("Invalid expected matrix dimensions for parameter ") + m_pp.resolveName(name).str +
                 std::string(" (at least one dimension must be specified)"));
      m_load_successful = false;
      return {loaded, false};
    }

    swap_t swap = NO_SWAP;
    if (rows <= 0) {
      std::swap(rows, cols);
      swap = SWAP;
    }
    return loadMatrixX(name, default_value, rows, cols, optional, unique, swap);
  }
  //}

  /* loadMatrixArray_internal helper function for loading an array of EigenXd matrices with known dimensions //{ */
  template <typename T>
  std::vector<ParamLoader::MatrixX<T>> ParamLoader::loadMatrixArray_internal(const std::string& name, const std::vector<MatrixX<T>>& default_value, optional_t optional, unique_t unique)
  {
    const auto resolved_name = m_pp.resolveName(name);
    int rows;
    std::vector<long int> cols;
    bool success = true;
    success = success && m_pp.getParam(resolved_name_t(resolved_name.str + "/rows"), rows);
    success = success && m_pp.getParam(resolved_name_t(resolved_name.str + "/cols"), cols);

    std::vector<MatrixX<T>> loaded;
    loaded.reserve(cols.size());

    int total_cols = 0;
    /* check correctness of loaded parameters so far calculate the total dimension //{ */

    if (!success) {
      printError(std::string("Failed to load ") + resolved_name.str + std::string("/rows or ") + resolved_name.str + std::string("/cols"));
      m_load_successful = false;
      return default_value;
    }
    if (rows < 0) {
      printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved_name.str + std::string(" (rows and cols must be >= 0)"));
      m_load_successful = false;
      return default_value;
    }
    for (const auto& col : cols) {
      if (col < 0) {
        printError(std::string("Invalid expected matrix dimensions for parameter ") + resolved_name.str + std::string(" (rows and cols must be >= 0)"));
        m_load_successful = false;
        return default_value;
      }
      total_cols += col;
    }

    //}

    const auto [loaded_matrix, loaded_ok] = loadMatrixX(name + "/data", MatrixX<T>(), rows, total_cols, optional, unique, NO_SWAP, false);
    /* std::cout << "loaded_matrix: " << loaded_matrix << std::endl; */
    /* std::cout << "loaded_matrix: " << loaded_matrix.rows() << "x" << loaded_matrix.cols() << std::endl; */
    /* std::cout << "expected dims: " << rows << "x" << total_cols << std::endl; */
    if (loaded_matrix.rows() != rows || loaded_matrix.cols() != total_cols) {
      m_load_successful = false;
      return default_value;
    }

    int cols_loaded = 0;
    for (unsigned it = 0; it < cols.size(); it++) {
      const int        cur_cols = cols.at(it);
      const MatrixX<T> cur_mat  = loaded_matrix.block(0, cols_loaded, rows, cur_cols);
      /* std::cout << "cur_mat: " << cur_mat << std::endl; */
      loaded.push_back(cur_mat);
      cols_loaded += cur_cols;
      printValue(resolved_name_t(resolved_name.str + "/matrix#" + std::to_string(it)), cur_mat);
    }
    return loaded;
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
  std::pair<T, bool> ParamLoader::load(const std::string& name, const T& default_value, optional_t optional, unique_t unique)
  {
    const auto resolved_name = m_pp.resolveName(name);
    T loaded = default_value;
    if (unique && check_duplicit_loading(resolved_name))
      return {loaded, false};

    bool cur_load_successful = true;
    // try to load the parameter
    const bool success = m_pp.getParam(resolved_name, loaded);
    if (!success) {
      // if it was not loaded, set the default value
      loaded = default_value;
      if (!optional) {
        // if the parameter was compulsory, alert the user and set the flag
        printError(std::string("Could not load non-optional parameter ") + resolved_name.str);
        cur_load_successful = false;
      }
    }

    if (cur_load_successful) {
      // everything is fine and just print the resolved_name and value if required
      if (m_print_values)
        printValue(resolved_name, loaded);
      // mark the param resolved_name as successfully loaded
      m_loaded_params.insert(resolved_name);
    } else {
      m_load_successful = false;
    }
    // finally, return the resulting value
    return {loaded, success};
  }
  //}
  //}

  /* loadParam function for optional parameters //{ */
  template <typename T>
  bool ParamLoader::loadParam(const std::string& name, T& out_value, const T& default_value)
  {
    const auto [ret, success] = load<T>(name, default_value, OPTIONAL, UNIQUE);
    out_value                 = ret;
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
  T ParamLoader::loadParam2(const std::string& name, const T& default_value)
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
   * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
   */
  template <typename T>
  bool ParamLoader::loadParamReusable(const std::string& name, T& out_value, const T& default_value)
  {
    const auto [ret, success] = load<T>(name, default_value, OPTIONAL, REUSABLE);
    out_value                 = ret;
    return success;
  }
  /*!
   * \brief Loads an optional reusable parameter from the rosparam server.
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
  T ParamLoader::loadParamReusable2(const std::string& name, const T& default_value)
  {
    const auto [ret, success] = load<T>(name, default_value, OPTIONAL, REUSABLE);
    return ret;
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
  bool ParamLoader::loadParam(const std::string& name, T& out_value)
  {
    const auto [ret, success] = load<T>(name, T(), COMPULSORY, UNIQUE);
    out_value                 = ret;
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
  T ParamLoader::loadParam2(const std::string& name)
  {
    const auto [ret, success] = load<T>(name, T(), COMPULSORY, UNIQUE);
    return ret;
  }
  /*!
   * \brief Loads a compulsory parameter from the rosparam server.
   *
   * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
   * the loading process is unsuccessful (loaded_successfully() will return false).
   * Using this method, the parameter can be loaded multiple times using the same ParamLoader instance without error.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
   */
  template <typename T>
  bool ParamLoader::loadParamReusable(const std::string& name, T& out_value)
  {
    const auto [ret, success] = load<T>(name, T(), COMPULSORY, REUSABLE);
    out_value                 = ret;
    return success;
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
  T ParamLoader::loadParamReusable2(const std::string& name)
  {
    const auto [ret, success] = load<T>(name, T(), COMPULSORY, REUSABLE);
    return ret;
  }
  //}

  /* loadParam specializations and convenience functions for Eigen dynamic matrix type //{ */

  /*!
   * \brief An overload for loading Eigen matrices.
   *
   * Matrix dimensions are deduced from the provided default value.
   * For compulsory Eigen matrices, use loadMatrixStatic(), loadMatrixKnown() or loadMatrixDynamic().
   * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
   * the default value is used.
   * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              True if loaded successfully, false otherwise.
   */
  template <typename T>
  bool ParamLoader::loadParam(const std::string& name, MatrixX<T>& mat, const MatrixX<T>& default_value)
  {
    const int  rows      = default_value.rows();
    const int  cols      = default_value.cols();
    const bool loaded_ok = loadMatrixDynamic(name, mat, default_value, rows, cols);
    return loaded_ok;
  }

  /*!
   * \brief An overload for loading Eigen matrices.
   *
   * Matrix dimensions are deduced from the provided default value.
   * For compulsory Eigen matrices, use loadMatrixStatic(), loadMatrixKnown() or loadMatrixDynamic().
   * If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or yaml config file),
   * the default value is used.
   * Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              The loaded parameter value.
   */
  template <typename T>
  ParamLoader::MatrixX<T> ParamLoader::loadParam2(const std::string& name, const MatrixX<T>& default_value)
  {
    MatrixX<T> ret;
    loadParam(name, ret, default_value);
    return ret;
  }

  //}

  // loadMatrixStatic function for loading of static Eigen::Matrices //{

  /*!
   * \brief Specialized method for loading compulsory Eigen matrix parameters.
   *
   * This variant assumes that the matrix dimensions are known in compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the loading process is unsuccessful. Using this method, the parameter can only be loaded once using the same ParamLoader instance
   * without error.
   *
   * \tparam rows  Expected number of rows of the matrix.
   * \tparam cols  Expected number of columns of the matrix.
   *
   * \param name  Name of the parameter in the rosparam server.
   * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \return      true if loaded successfully, false otherwise.
   *
   */
  template <int rows, int cols, typename T>
  bool ParamLoader::loadMatrixStatic(const std::string& name, Eigen::Matrix<T, rows, cols>& mat)
  {
    const auto [ret, loaded_ok] = loadMatrixStatic_internal<rows, cols, T>(name, Eigen::Matrix<T, rows, cols>::Zero(), COMPULSORY, UNIQUE);
    mat                         = ret;
    return loaded_ok;
  }

  /*!
   * \brief Specialized method for loading Eigen matrix parameters with default value.
   *
   * This variant assumes that the matrix dimensions are known in compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the default value is used. Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \tparam rows          Expected number of rows of the matrix.
   * \tparam cols          Expected number of columns of the matrix.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param mat           Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
   *
   */
  template <int rows, int cols, typename T, typename Derived>
  bool ParamLoader::loadMatrixStatic(const std::string& name, Eigen::Matrix<T, rows, cols>& mat, const Eigen::MatrixBase<Derived>& default_value)
  {
    const auto [ret, loaded_ok] = loadMatrixStatic_internal<rows, cols, T>(name, Eigen::Matrix<T, rows, cols>(default_value), OPTIONAL, UNIQUE);
    mat                         = ret;
    return loaded_ok;
  }

  /*!
   * \brief Specialized method for loading compulsory Eigen matrix parameters.
   *
   * This variant assumes that the matrix dimensions are known in compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the loading process is unsuccessful. Using this method, the parameter can only be loaded once using the same ParamLoader instance
   * without error.
   *
   * \tparam rows  Expected number of rows of the matrix.
   * \tparam cols  Expected number of columns of the matrix.
   *
   * \param name  Name of the parameter in the rosparam server.
   * \return      The loaded parameter value.
   *
   */
  template <int rows, int cols, typename T>
  Eigen::Matrix<T, rows, cols> ParamLoader::loadMatrixStatic2(const std::string& name)
  {
    Eigen::Matrix<T, rows, cols> ret;
    loadMatrixStatic(name, ret);
    return ret;
  }

  /*!
   * \brief Specialized method for loading Eigen matrix parameters with default value.
   *
   * This variant assumes that the matrix dimensions are known in compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the default value is used. Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \tparam rows          Expected number of rows of the matrix.
   * \tparam cols          Expected number of columns of the matrix.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              The loaded parameter value.
   *
   */
  template <int rows, int cols, typename T, typename Derived>
  Eigen::Matrix<T, rows, cols> ParamLoader::loadMatrixStatic2(const std::string& name, const Eigen::MatrixBase<Derived>& default_value)
  {
    Eigen::Matrix<T, rows, cols> ret;
    loadMatrixStatic(name, ret, default_value);
    return ret;
  }
  //}

  // loadMatrixKnown function for loading of Eigen matrices with known dimensions //{

  /*!
   * \brief Specialized method for loading compulsory Eigen matrix parameters.
   *
   * This variant assumes that the matrix dimensions are known in runtime, but not compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the loading process is unsuccessful. Using this method, the parameter can only be loaded once using the same ParamLoader instance
   * without error.
   *
   * \param name  Name of the parameter in the rosparam server.
   * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param rows  Expected number of rows of the matrix.
   * \param cols  Expected number of columns of the matrix.
   * \return      true if loaded successfully, false otherwise.
   */
  template <typename T>
  bool ParamLoader::loadMatrixKnown(const std::string& name, MatrixX<T>& mat, int rows, int cols)
  {
    const auto [ret, loaded_ok] = loadMatrixKnown_internal(name, MatrixX<T>(), rows, cols, COMPULSORY, UNIQUE);
    mat                         = ret;
    return loaded_ok;
  }

  /*!
   * \brief Specialized method for loading Eigen matrix parameters with default value.
   *
   * This variant assumes that the matrix dimensions are known in runtime, but not compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the default value is used. Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param mat           Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \param rows          Expected number of rows of the matrix.
   * \param cols          Expected number of columns of the matrix.
   * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
   */
  template <typename T, typename Derived>
  bool ParamLoader::loadMatrixKnown(const std::string& name, MatrixX<T>& mat, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols)
  {
    const auto [ret, loaded_ok] = loadMatrixKnown_internal(name, MatrixX<T>(default_value), rows, cols, OPTIONAL, UNIQUE);
    mat                         = ret;
    return loaded_ok;
  }

  /*!
   * \brief Specialized method for loading compulsory Eigen matrix parameters.
   *
   * This variant assumes that the matrix dimensions are known in runtime, but not compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the loading process is unsuccessful. Using this method, the parameter can only be loaded once using the same ParamLoader instance
   * without error.
   *
   * \param name  Name of the parameter in the rosparam server.
   * \param rows  Expected number of rows of the matrix.
   * \param cols  Expected number of columns of the matrix.
   * \return      The loaded parameter value.
   */
  template <typename T>
  ParamLoader::MatrixX<T> ParamLoader::loadMatrixKnown2(const std::string& name, int rows, int cols)
  {
    MatrixX<T> ret;
    loadMatrixKnown(name, ret, rows, cols);
    return ret;
  }

  /*!
   * \brief Specialized method for loading Eigen matrix parameters with default value.
   *
   * This variant assumes that the matrix dimensions are known in runtime, but not compiletime.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the default value is used. Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \param rows          Expected number of rows of the matrix.
   * \param cols          Expected number of columns of the matrix.
   * \return              The loaded parameter value.
   */
  template <typename T, typename Derived>
  ParamLoader::MatrixX<T> ParamLoader::loadMatrixKnown2(const std::string& name, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols)
  {
    MatrixX<T> ret;
    loadMatrixKnown(name, ret, default_value, rows, cols);
    return ret;
  }
  //}

  // loadMatrixDynamic function for half-dynamic loading of MatrixX<T> //{

  /*!
   * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
   *
   * This variant assumes that the only one of the matrix dimensions are known, the other is selected based on the loaded value.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the loading process is unsuccessful. Using this method, the parameter can only be loaded once using the same ParamLoader instance
   * without error.
   *
   * \param name  Name of the parameter in the rosparam server.
   * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param rows  Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of columns
   * and the size of the loaded array). \param cols  Expected number of columns of the matrix (negative value indicates that the number of columns is to be
   * deduced from the specified number of rows and the size of the loaded array). \return      true if loaded successfully, false otherwise.
   */
  template <typename T>
  bool ParamLoader::loadMatrixDynamic(const std::string& name, MatrixX<T>& mat, int rows, int cols)
  {
    const auto [ret, loaded_ok] = loadMatrixDynamic_internal(name, MatrixX<T>(), rows, cols, COMPULSORY, UNIQUE);
    mat                         = ret;
    return loaded_ok;
  }

  /*!
   * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
   *
   * This variant assumes that the only one of the matrix dimensions are known, the other is selected based on the loaded value.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the default value is used. Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \param mat           Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param rows          Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of
   * columns and the size of the loaded array). \param cols          Expected number of columns of the matrix (negative value indicates that the number of
   * columns is to be deduced from the specified number of rows and the size of the loaded array). \return              true if the parameter was loaded from \p
   * rosparam, false if the default value was used.
   */
  template <typename T, typename Derived>
  bool ParamLoader::loadMatrixDynamic(const std::string& name, MatrixX<T>& mat, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols)
  {
    const auto [ret, loaded_ok] = loadMatrixDynamic_internal(name, MatrixX<T>(default_value), rows, cols, OPTIONAL, UNIQUE);
    mat                         = ret;
    return loaded_ok;
  }

  /*!
   * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
   *
   * This variant assumes that the only one of the matrix dimensions are known, the other is selected based on the loaded value.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the loading process is unsuccessful. Using this method, the parameter can only be loaded once using the same ParamLoader instance
   * without error.
   *
   * \param name  Name of the parameter in the rosparam server.
   * \param rows  Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of columns
   * and the size of the loaded array). \param cols  Expected number of columns of the matrix (negative value indicates that the number of columns is to be
   * deduced from the specified number of rows and the size of the loaded array). \return      The loaded parameter value.
   */
  template <typename T>
  ParamLoader::MatrixX<T> ParamLoader::loadMatrixDynamic2(const std::string& name, int rows, int cols)
  {
    MatrixX<T> ret;
    loadMatrixDynamic(name, ret, rows, cols);
    return ret;
  }

  /*!
   * \brief Specialized method for loading compulsory dynamic Eigen matrix parameters.
   *
   * This variant assumes that the only one of the matrix dimensions are known, the other is selected based on the loaded value.
   * If the dimensions of the loaded matrix do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the default value is used. Using this method, the parameter can only be loaded once using the same ParamLoader instance without error.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \param rows          Expected number of rows of the matrix (negative value indicates that the number of rows is to be deduced from the specified number of
   * columns and the size of the loaded array). \param cols          Expected number of columns of the matrix (negative value indicates that the number of
   * columns is to be deduced from the specified number of rows and the size of the loaded array). \return              The loaded parameter value.
   */
  template <typename T, typename Derived>
  ParamLoader::MatrixX<T> ParamLoader::loadMatrixDynamic2(const std::string& name, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols)
  {
    MatrixX<T> ret;
    loadMatrixDynamic(name, ret, default_value, rows, cols);
    return ret;
  }

  //}

  // loadMatrixArray function for loading of an array of MatrixX<T> with known dimensions //{
  /*!
   * \brief Specialized method for loading compulsory parameters, interpreted as an array of dynamic Eigen matrices.
   *
   * The number of rows and columns of the matrices to be loaded is specified in the \c rosparam parameter. Specifically, the \c name/rows value specifies the
   * number of rows, which must be common to all the loaded matrices (i.e. it is one integer >= 0), and the \c name/cols value specifies the number of columns
   * of each matrix (i.e. it is an array of integers > 0). The \c name/data array contains the values of the elements of the matrices and it must have length
   * \f$ r\sum_i c_i \f$, where \f$ r \f$ is the common number of rows and \f$ c_i \f$ is the number of columns of the \f$ i \f$-th matrix.
   * A typical structure of a \c yaml file, specifying the
   * matrix array to be loaded using this method, is
   *
   * \code{.yaml}
   *
   * matrix_array:
   *   rows: 3
   *   cols: [1, 2]
   *   data: [-5.0, 0.0, 23.0,
   *          -5.0, 0.0, 12.0,
   *           2.0,   4.0,  7.0]
   *
   * \endcode
   *
   * which will be loaded as a \c std::vector, containing one \f$ 3\times 1 \f$ matrix and one \f$ 3\times 2 \f$ matrix.
   *
   * If the dimensions of the loaded matrices do not match the specified number of rows and columns, the loading process is unsuccessful (loaded_successfully()
   * will return false). If the parameter with the specified name is not found on the \c rosparam server (e.g. because it is not specified in the launchfile or
   * yaml config file), the loading process is unsuccessful. Using this method, the parameter can only be loaded once using the same ParamLoader instance
   * without error.
   *
   * \param name  Name of the parameter in the rosparam server.
   * \param mat   Reference to the variable to which the parameter value will be stored (such as a class member variable).
   *
   */
  template <typename T>
  void ParamLoader::loadMatrixArray(const std::string& name, std::vector<MatrixX<T>>& mat)
  {
    mat = loadMatrixArray2<double>(name);
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
  template <typename T>
  void ParamLoader::loadMatrixArray(const std::string& name, std::vector<MatrixX<T>>& mat, const std::vector<MatrixX<T>>& default_value)
  {
    mat = loadMatrixArray2(name, default_value);
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
  template <typename T>
  std::vector<ParamLoader::MatrixX<T>> ParamLoader::loadMatrixArray2(const std::string& name)
  {
    return loadMatrixArray_internal(name, std::vector<MatrixX<T>>(), COMPULSORY, UNIQUE);
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
  template <typename T>
  std::vector<ParamLoader::MatrixX<T>> ParamLoader::loadMatrixArray2(const std::string& name, const std::vector<MatrixX<T>>& default_value)
  {
    return loadMatrixArray_internal(name, default_value, OPTIONAL, UNIQUE);
  }
  //}

}  // namespace mrs_lib
