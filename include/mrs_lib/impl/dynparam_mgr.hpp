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

  template <typename MemT>
  bool DynparamMgr::register_param(const std::string& name, MemT* param_var)
  {
    // make sure that the parameter is always declared in ROS
    // even if the default value will be loaded from YAML (by default,
    // when loading from YAML, the ParamProvider does not declare the
    // parameter in ROS to speed up the loading)
    if (!m_node->has_parameter(m_pp.resolveName(name)))
    {
      const bool declare_success = m_pp.declareParam<MemT>(name, true);
      if (!declare_success)
        return false;
    }

    const bool get_success = m_pp.getParam(name, *param_var, true);
    if (!get_success)
      return false;

    m_registered_params.emplace_back(
          name,
          to_param_type<MemT>(),
          param_var
        );
    return true;
  }

  struct DynparamMgr::registered_param_t
  {
    std::string name;
    rclcpp::ParameterType type;
    valid_types_t param_ptr;

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
};
