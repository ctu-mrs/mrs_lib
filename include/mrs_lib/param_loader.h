// clang: MatousFormat
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_set>
#include <Eigen/Dense>
#include <std_msgs/msg/color_rgba.hpp>
#include <mrs_lib/param_provider.h>

namespace mrs_lib
{

  /*!
   * \brief Helper overload for printing of Eigen matrices.
   *
   * \param os          The output stream to send the printed matrix to.
   * \param var         The matrix to be printed.
   * \return            A reference to the output stream.
   */
  template <typename T>
  std::ostream& operator<<(std::ostream& os, const Eigen::MatrixX<T>& var);

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
 * See documentation of ROS launchfiles here: http://wiki.rclcpp.org/roslaunch/XML.
 * Specifically, the `param` XML tag is used for loading parameters directly from the launchfile: http://wiki.rclcpp.org/roslaunch/XML/param,
 * and the `rosparam` XML tag tag is used for loading parameters from a `yaml` file: http://wiki.rclcpp.org/roslaunch/XML/rosparam.
 *
 */
class ParamLoader {

private:
  enum unique_t
  {
    UNIQUE   = true,
    REUSABLE = false
  };
  enum optional_t
  {
    OPTIONAL   = true,
    COMPULSORY = false
  };
  enum swap_t
  {
    SWAP    = true,
    NO_SWAP = false
  };

  template <typename T>
  using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

  using resolved_name_t = ParamProvider::resolved_name_t;

private:
  bool                                m_load_successful, m_print_values;
  std::string                         m_node_name;
  std::shared_ptr<rclcpp::Node>       m_node;
  mrs_lib::ParamProvider              m_pp;
  std::unordered_set<resolved_name_t> m_loaded_params;

  /* printing helper functions //{ */

  /* printError and printWarning functions //{*/

  void printError(const std::string& str);
  void printWarning(const std::string& str);

  //}

  /* printValue function and overloads //{ */

  template <typename T>
  void printValue(const resolved_name_t& name, const T& value) const;
  
  //}

  //}

  /* loading helper functions //{ */

  /* check_duplicit_loading checks whether the parameter was already loaded - returns true if yes //{ */

  bool check_duplicit_loading(const resolved_name_t& name);
  
  //}

  /* helper functions for loading dynamic Eigen matrices //{ */
  // loadMatrixX helper function for loading dynamic Eigen matrices //{
  template <typename T>
  std::pair<MatrixX<T>, bool> loadMatrixX(const std::string& name, const MatrixX<T>& default_value, int rows, int cols = Eigen::Dynamic, optional_t optional = OPTIONAL, unique_t unique = UNIQUE, swap_t swap = NO_SWAP, bool printValues = true);
  //}

  /* loadMatrixStatic_internal helper function for loading static Eigen matrices //{ */
  template <int rows, int cols, typename T>
  std::pair<Eigen::Matrix<T, rows, cols>, bool> loadMatrixStatic_internal(const std::string& name, const Eigen::Matrix<T, rows, cols>& default_value, optional_t optional, unique_t unique);
  //}

  /* loadMatrixKnown_internal helper function for loading EigenXd matrices with known dimensions //{ */
  template <typename T>
  std::pair<MatrixX<T>, bool> loadMatrixKnown_internal(const std::string& name, const MatrixX<T>& default_value, int rows, int cols, optional_t optional, unique_t unique);
  //}

  /* loadMatrixDynamic_internal helper function for loading Eigen matrices with one dynamic (unspecified) dimension //{ */
  template <typename T>
  std::pair<MatrixX<T>, bool> loadMatrixDynamic_internal(const std::string& name, const MatrixX<T>& default_value, int rows, int cols, optional_t optional, unique_t unique);
  //}

  /* loadMatrixArray_internal helper function for loading an array of EigenXd matrices with known dimensions //{ */
  template <typename T>
  std::vector<MatrixX<T>> loadMatrixArray_internal(const std::string& name, const std::vector<MatrixX<T>>& default_value, optional_t optional, unique_t unique);
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
  std::pair<T, bool> load(const std::string& name, const T& default_value, optional_t optional = OPTIONAL, unique_t unique = UNIQUE);
  //}
  //}

public:
  /*!
   * \brief Main constructor.
   *
   * \param nh            The parameters will be loaded from rosparam using this node handle.
   * \param printValues  If true, the loaded values will be printed to stdout using std::cout or ROS_INFO if node_name is not empty.
   * \param node_name     Optional node name used when printing the loaded values or loading errors.
   */
  ParamLoader(const std::shared_ptr<rclcpp::Node>& node, bool printValues = true, std::string_view node_name = std::string());

  /* Constructor overloads //{ */
  /*!
   * \brief Convenience overload to enable writing ParamLoader pl(nh, node_name);
   *
   * \param nh            The parameters will be loaded from rosparam using this node handle.
   * \param node_name     Optional node name used when printing the loaded values or loading errors.
   */
  ParamLoader(const std::shared_ptr<rclcpp::Node>& node, std::string node_name);

  //}

  /* setPrefix function //{ */

  /*!
   * \brief Sets a prefix that will be applied to parameter names before subnode namespaces.
   *
   * The prefix will be applied as-is, so if you need to separate it from the parameter name
   * e.g. using a forward slash '/', you must include it in the prefix.
   *
   * \param prefix      The prefix to be applied to all parameter names.
   */
  void setPrefix(const std::string& prefix);

  //}

  /* getPrefix function //{ */

  /*!
   * \brief Returns the current parameter name prefix.
   *
   * \return the current prefix to be applied to the loaded parameters.
   */
  std::string getPrefix();

  //}

  /* addYamlFile() function //{ */

  /*!
   * \brief Adds the specified file as a source of static parameters.
   *
   * \param filepath The full path to the yaml file to be loaded.
   * \return true if loading and parsing the file was successful, false otherwise.
   */
  bool addYamlFile(const std::string& filepath);
  //}

  /* addYamlFileFromParam() function //{ */

  /*!
   * \brief Loads a filepath from a parameter loads that file as a YAML.
   *
   * \param param_name Name of the parameter from which to load the YAML filename to be loaded.
   * \return true      if loading and parsing the file was successful, false otherwise.
   */
  bool addYamlFileFromParam(const std::string& param_name);
  //}

  /*!
   * \brief Copies parsed YAMLs from another ParamLoader.
   *
   * \param param_loader The ParamLoader object to copy the YAML files from.
   */
  void copyYamls(const ParamLoader& param_loader);

  /*!
   * \brief Returns the internal ParamProvider object.
   *
   * \return  a reference to the internal ParamProvider object.
   */
  ParamProvider& getParamProvider();

  /* loadedSuccessfully function //{ */
  /*!
   * \brief Indicates whether all compulsory parameters were successfully loaded.
   *
   * \return false if any compulsory parameter was not loaded (is not present at rosparam server). Otherwise returns true.
   */
  bool loadedSuccessfully() const;
  //}

  /* resetLoadedSuccessfully function //{ */
  /*!
   * \brief Resets the loadedSuccessfully flag back to true.
   */
  void resetLoadedSuccessfully();
  //}

  /* resetUniques function //{ */
  /*!
   * \brief Resets the list of already loaded parameter names used when checking for uniqueness.
   */
  void resetUniques();
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
  bool loadParam(const std::string& name, T& out_value, const T& default_value);

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
  T loadParam2(const std::string& name, const T& default_value);

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
  bool loadParamReusable(const std::string& name, T& out_value, const T& default_value);

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
  T loadParamReusable2(const std::string& name, const T& default_value);

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
  bool loadParam(const std::string& name, T& out_value); 

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
  T loadParam2(const std::string& name);

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
  bool loadParamReusable(const std::string& name, T& out_value);

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
  T loadParamReusable2(const std::string& name);

  //}

  /* loadParam specializations for rclcpp::Duration type //{ */

  /*!
   * \brief An overload for loading rclcpp::Duration.
   *
   * The duration will be loaded as a \p double, representing a number of seconds, and then converted to rclcpp::Duration.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
   */
  bool loadParam(const std::string& name, rclcpp::Duration& out, const rclcpp::Duration& default_value);

  /*!
   * \brief An overload for loading rclcpp::Duration.
   *
   * The duration will be loaded as a \p double, representing a number of seconds, and then converted to rclcpp::Duration.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
   */
  bool loadParam(const std::string& name, rclcpp::Duration& out);

  //}

  /* loadParam specializations for std_msgs::msg::ColorRGBA type //{ */

  /*!
   * \brief An overload for loading std_msgs::msg::ColorRGBA.
   *
   * The color will be loaded as several \p double -typed variables, representing the R, G, B and A color elements.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param out_value     Reference to the variable to which the parameter value will be stored (such as a class member variable).
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              true if the parameter was loaded from \p rosparam, false if the default value was used.
   */
  bool loadParam(const std::string& name, std_msgs::msg::ColorRGBA& out, const std_msgs::msg::ColorRGBA& default_value = std_msgs::msg::ColorRGBA());

  /*!
   * \brief An overload for loading std_msgs::msg::ColorRGBA.
   *
   * The color will be loaded as several \p double -typed variables, representing the R, G, B and A color elements.
   *
   * \param name          Name of the parameter in the rosparam server.
   * \param default_value This value will be used if the parameter name is not found in the rosparam server.
   * \return              The loaded parameter value.
   */
  std_msgs::msg::ColorRGBA loadParam2(const std::string& name, const std_msgs::msg::ColorRGBA& default_value = std_msgs::msg::ColorRGBA());

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
  bool loadParam(const std::string& name, MatrixX<T>& mat, const MatrixX<T>& default_value);

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
  MatrixX<T> loadParam2(const std::string& name, const MatrixX<T>& default_value);

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
  bool loadMatrixStatic(const std::string& name, Eigen::Matrix<T, rows, cols>& mat);

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
  bool loadMatrixStatic(const std::string& name, Eigen::Matrix<T, rows, cols>& mat, const Eigen::MatrixBase<Derived>& default_value);

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
  template <int rows, int cols, typename T = double>
  Eigen::Matrix<T, rows, cols> loadMatrixStatic2(const std::string& name);

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
  Eigen::Matrix<T, rows, cols> loadMatrixStatic2(const std::string& name, const Eigen::MatrixBase<Derived>& default_value);

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
  bool loadMatrixKnown(const std::string& name, MatrixX<T>& mat, int rows, int cols);

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
  bool loadMatrixKnown(const std::string& name, MatrixX<T>& mat, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols);

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
  template <typename T = double>
  MatrixX<T> loadMatrixKnown2(const std::string& name, int rows, int cols);

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
  MatrixX<T> loadMatrixKnown2(const std::string& name, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols);

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
  bool loadMatrixDynamic(const std::string& name, MatrixX<T>& mat, int rows, int cols);

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
  bool loadMatrixDynamic(const std::string& name, MatrixX<T>& mat, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols);

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
  template <typename T = double>
  MatrixX<T> loadMatrixDynamic2(const std::string& name, int rows, int cols);

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
  MatrixX<T> loadMatrixDynamic2(const std::string& name, const Eigen::MatrixBase<Derived>& default_value, int rows, int cols);

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
  void loadMatrixArray(const std::string& name, std::vector<MatrixX<T>>& mat);

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
  void loadMatrixArray(const std::string& name, std::vector<MatrixX<T>>& mat, const std::vector<MatrixX<T>>& default_value);

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
  template <typename T = double>
  std::vector<MatrixX<T>> loadMatrixArray2(const std::string& name);

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
  std::vector<MatrixX<T>> loadMatrixArray2(const std::string& name, const std::vector<MatrixX<T>>& default_value);
  
  //}

};
//}

/*!
 * \brief An overload for loading rclcpp::Duration.
 *
 * The duration will be loaded as a \p double, representing a number of seconds, and then converted to rclcpp::Duration.
 *
 * \param name          Name of the parameter in the rosparam server.
 * \param default_value This value will be used if the parameter name is not found in the rosparam server.
 * \return              The loaded parameter value.
 */
template <>
rclcpp::Duration ParamLoader::loadParam2<rclcpp::Duration>(const std::string& name, const rclcpp::Duration& default_value);

/*!
 * \brief An overload for loading rclcpp::Duration.
 *
 * The duration will be loaded as a \p double, representing a number of seconds, and then converted to rclcpp::Duration.
 *
 * \param name          Name of the parameter in the rosparam server.
 * \return              The loaded parameter value.
 */
template <>
rclcpp::Duration ParamLoader::loadParam2<rclcpp::Duration>(const std::string& name);

}  // namespace mrs_lib

#include <mrs_lib/impl/param_loader.hpp>
