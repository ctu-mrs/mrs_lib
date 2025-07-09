// clang: MatousFormat
/**  \file
     \brief Defines DynparamMgr - a convenience class for managing dynamic ROS parameters.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#pragma once

#include <concepts>

#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/param_provider.h>

namespace mrs_lib
{

  /**
   * \brief Convenience class for managing dynamic ROS parameters.
   *
   * In the typical use-case, you give DynparamMgr a mutex and register a number of ROS parameters
   * together with pointers to variables that you want to keep up-to-date with the value of the respective
   * parameters. When a parameter is changed, the mutex is locked and the value of the corresponding
   * variable is updated accordingly.
   *
   * Default values of the parameters can be loaded from ROS or from YAML.
   *
   */
  class DynparamMgr
  {

  public:
  /*!
   * \brief A convenience alias defining the valid C++ types to be registered.
   *
   */
    using valid_types_t = std::tuple<
        int,
        int64_t,
        float,
        double,
        bool,
        std::string,
        std::vector<uint8_t>,
        std::vector<int64_t>,
        std::vector<double>,
        std::vector<bool>,
        std::vector<std::string>
      >;

    template <typename T>
    using update_cbk_t = std::function<void(const T&)>;

  /*!
   * \brief The main constructor.
   *
   * Constructs the DynparamMgr object and registers the `on_set_parameters` callback.
   *
   * \param node      A pointer to the ROS node handle used for registering, getting, setting, etc. of the parameters.
   * \param mtx       The mutex to be locked whenever the pointed-to variables passed in register_param() are touched.
   */
    DynparamMgr(const std::shared_ptr<rclcpp::Node>& node, std::mutex& mtx);

  /*!
   * \brief Registers a ROS parameter and a corresponding variable.
   *
   * This method registers a pair of the provided pointer to a variable and a name of a ROS parameter to keep the variable up-to-date with the value of the ROS parameter.
   *
   * It will:
   * 1. Declare the parameter in ROS as a dynamic parameter with the corresponding type .
   * 2. Load its default value using ParamProvider - this means the default value can be loaded from ROS or directly from a provided YAML file, if available.
   * 3. Set the value of the variable pointed-to by param_var to this default value.
   * 4. Save the name of the parameter and the pointer so that the variable can be updated when the `on_set_parameters` is called.
   * 
   * \param name        Name of the parameter (this will be used for the declaration etc.).
   * \param param_var   Pointer to a variable whose value will be changed when the parameter is updated. It is the user's responsibility to ensure that the lifetime of the pointed-to variable is valid while it is being used by the DynparamMgr.
   * \param update_cbk  Optional function to be called when the parameter is changed.
   * \return            true if the parameter was successfully declared and its default value loaded.
   *
   * \note The C++ type passed to this function will be mapped to one of the valid values of rclcpp::ParameterType. Not all C++ types are supported (see valid_types_t).
   */
    template <typename MemT>
    bool register_param(const std::string& name, MemT* param_var, const update_cbk_t<MemT>& update_cbk = {});

  /*!
   * \brief An overload of register_param() for specifying minimum and maximum of a number value.
   *
   * \param name        Name of the parameter (this will be used for the declaration etc.).
   * \param param_var   Pointer to a variable whose value will be changed when the parameter is updated. It is the user's responsibility to ensure that the lifetime of the pointed-to variable is valid while it is being used by the DynparamMgr.
   * \param minimum     The minimal value of the number (a lower value cannot be set).
   * \param maximum     The maximal value of the number (a higher value cannot be set).
   * \param update_cbk  Optional function to be called when the parameter is changed.
   * \return            true if the parameter was successfully declared and its default value loaded.
   */
    template <typename MemT>
    bool register_param(const std::string& name, MemT* param_var, const MemT minimum, const MemT maximum, const update_cbk_t<MemT>& update_cbk = {})
    requires std::integral<MemT> or std::floating_point<MemT>;

  /*!
   * \brief Pushes the current values of the pointed-to variables to ROS.
   *
   * This method serves to update changes to the registered pointed-to variables made in your code to their ROS counterparts.
   * \return    the result of setting the parameters returned by the internally called rclcpp::Node::set_parameters_atomically().
   */
    rcl_interfaces::msg::SetParametersResult update_to_ros();

  /*!
   * \brief Returns the underlying ParamProvider object.
   *
   * Useful e.g. for manually setting values of some parameters while making sure the correct
   * subnode namespacing is used etc.
   * \return    a reference to the underlying ParamProvider object.
   */
    mrs_lib::ParamProvider& get_param_provider()
    {
      return m_pp;
    };

  private:
    std::shared_ptr<rclcpp::Node> m_node;
    mrs_lib::ParamProvider m_pp;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_cbk;

    rcl_interfaces::msg::SetParametersResult cbk_param_update(const std::vector<rclcpp::Parameter>& parameters);

    struct registered_param_t;
    std::mutex& m_mtx;
    std::vector<registered_param_t> m_registered_params;
  };

}  // namespace mrs_lib

#include <mrs_lib/impl/dynparam_mgr.hpp>
