// clang: MatousFormat
/**  \file
     \brief Defines DynparamMgr - a convenience class for managing dynamic ROS parameters.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#pragma once

#include <mrs_lib/dynparam_mgr.h>
#include <any>

namespace mrs_lib
{

  /* register_param() method //{ */
  template <typename MemT>
  bool DynparamMgr::register_param(const std::string& name, MemT* param_var, const std::function<void(const MemT&)>& update_cbk)
  {
    const auto resolved_name = m_pp.resolveName(name);
    // make sure that the parameter is always declared in ROS
    // even if the default value will be loaded from YAML (by default,
    // when loading from YAML, the ParamProvider does not declare the
    // parameter in ROS to speed up the loading)
    if (!m_node->has_parameter(resolved_name))
    {
      const bool declare_success = m_pp.declareParam<MemT>(resolved_name, true);
      if (!declare_success)
        return false;
    }
  
    // load the param variable with the current value of the parameter
    const bool get_success = m_pp.getParam(resolved_name, *param_var, true);
    if (!get_success)
      return false;
  
    // remember the registered parameter, the corresponding variable, etc.
    m_registered_params.emplace_back(
          *m_node,
          name,
          to_param_type<MemT>(),
          param_var,
          update_cbk
        );
    return true;
  }
  //}

  template <typename MemT>
  bool DynparamMgr::register_param(const std::string& name, MemT* param_var, const MemT minimum, const MemT maximum, const std::function<void(const MemT&)>& update_cbk)
  requires std::integral<MemT> or std::floating_point<MemT>
  {
    const auto resolved_name = m_pp.resolveName(name);
    // make sure that the parameter is always declared in ROS
    // even if the default value will be loaded from YAML (by default,
    // when loading from YAML, the ParamProvider does not declare the
    // parameter in ROS to speed up the loading)
    if (!m_node->has_parameter(resolved_name))
    {
      const bool declare_success = m_pp.declareParam<MemT>(resolved_name, minimum, maximum, true);
      if (!declare_success)
        return false;
    }
  
    // load the param variable with the current value of the parameter
    const bool get_success = m_pp.getParam(resolved_name, *param_var, true);
    if (!get_success)
      return false;
  
    // remember the registered parameter, the corresponding variable, etc.
    m_registered_params.emplace_back(
          *m_node,
          name,
          to_param_type<MemT>(),
          param_var,
          update_cbk
        );
    return true;
  }

  /* registered_param_t struct //{ */
  
  struct DynparamMgr::registered_param_t
  {
    rclcpp::Node& node; // non-owning reference (the memory is share-owned by the DynparamMgr)
    std::string name;
    rclcpp::ParameterType type;
    valid_types_t param_ptr;
    valid_callbacks_t update_cbk;
  
    template <typename T>
    bool try_cast(T& out)
    {
      try
      {
        out = std::get<T>(param_ptr);
        return true;
      }
      catch (const std::bad_any_cast& e)
      {
        return false;
      }
    }
  
    template <typename NewValueT>
    bool update_value(const NewValueT& new_value)
    {
      // if there is an update callback registered, try to call it
      std::visit([&new_value, this](auto arg)
        {
          using CbkT = decltype(arg);
          if constexpr (std::is_invocable_v<CbkT, NewValueT>)
          {
            if (arg)
            // actually call the callback
              arg(new_value);
          }
          else
          {
            RCLCPP_ERROR_STREAM_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "Cannot call update callback for parameter \"" << name << "\" - incompatible callback type!");
          }
        }, update_cbk);

      return std::visit([&new_value](auto arg)
        {
          using ParamT = std::remove_pointer_t<decltype(arg)>;
          if constexpr (std::is_convertible_v<NewValueT, ParamT>)
          {
            *arg = static_cast<ParamT>(new_value);
            return true;
          }
          else
            return false;
        }, param_ptr);
    }
  
    rclcpp::ParameterValue to_param_val() const;
  };
  
  //}

};
