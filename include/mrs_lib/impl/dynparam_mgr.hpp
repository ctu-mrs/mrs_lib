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
  bool DynparamMgr::register_param(const std::string& name, MemT* param_var, const update_cbk_t<MemT>& update_cbk)
  {
    return register_param_impl<MemT>(name, param_var, std::nullopt, std::nullopt, update_cbk);
  }

  template <typename MemT>
  bool DynparamMgr::register_param(const std::string& name, MemT* param_var, const MemT& default_value, const update_cbk_t<MemT>& update_cbk)
  {
    return register_param_impl<MemT>(name, param_var, default_value, std::nullopt, update_cbk);
  }

  template <typename MemT>
  bool DynparamMgr::register_param(const std::string& name, MemT* param_var, const range_t<MemT>& valid_range, const update_cbk_t<MemT>& update_cbk)
  requires(numeric<MemT>)
  {
    return register_param_impl<MemT>(name, param_var, std::nullopt, valid_range, update_cbk);
  }

  template <typename MemT>
  bool DynparamMgr::register_param(const std::string& name, MemT* param_var, const MemT& default_value, const range_t<MemT>& valid_range, const update_cbk_t<MemT>& update_cbk)
  requires(numeric<MemT>)
  {
    return register_param_impl<MemT>(name, param_var, default_value, valid_range, update_cbk);
  }

  template <typename MemT>
  bool DynparamMgr::register_param_impl(const std::string& name, MemT* param_var, const std::optional<MemT>& default_value, const std::optional<range_t<MemT>>& valid_range, const update_cbk_t<MemT>& update_cbk)
  {
    const auto resolved_name = m_pp.resolveName(name);

    ParamProvider::get_options_t<MemT> opts;
    opts.always_declare = true;
    opts.declare_options.reconfigurable = true;
    opts.declare_options.default_value = default_value;
    opts.declare_options.range = valid_range;

    // load the current value of the parameter
    MemT current_value;
    const bool get_success = m_pp.getParam(resolved_name, current_value, opts);
    if (!get_success)
    {
      m_load_successful = false;
      RCLCPP_ERROR_STREAM(m_node->get_logger(), "[" << m_node->get_name() << "]: Could not register dynamic parameter '" << name << "'");
      return false;
    }

    // only assign the default value if everything is OK, otherwise leave param_var untouched
    *param_var = current_value;

    // remember the registered parameter, the corresponding variable, etc.
    RCLCPP_INFO_STREAM(m_node->get_logger(), "[" << m_node->get_name() << "]: Dynamic parameter '" << name << "':\t" << *param_var);

    m_registered_params.emplace_back(
          *m_node,
          resolved_name,
          to_param_type<MemT>(),
          param_var,
          update_cbk
        );
    return true;
  }
  //}

  /* registered_param_t struct //{ */
  
  struct DynparamMgr::registered_param_t
  {
    // | --------------------- Metaprogramming -------------------- |
    // some metaprogramming tooling to generate a variant of pointers from a tuple of types
    template <typename ... Types>
    using variant_of_pointers_t = std::variant<std::add_pointer_t<Types>...>;

    template <typename Type>
    struct pointer_variant_from_list_t;

    template <typename ... Types>
    struct pointer_variant_from_list_t<std::tuple<Types...>>
    {
      using type = variant_of_pointers_t<Types...>;
    };

    // the same tooling for creating a variant of functions with the types as const-ref arguments
    template <typename Type>
    using add_function_cref_t = std::function<void(const Type&)>;

    template <typename ... Types>
    using variant_of_functions_t = std::variant<add_function_cref_t<Types>...>;

    template <typename Type>
    struct function_variant_from_list_t;

    template <typename ... Types>
    struct function_variant_from_list_t<std::tuple<Types...>>
    {
      using type = variant_of_functions_t<Types...>;
    };

    // define the std::variant of pointers-to the types from valid_types_t
    using param_ptr_variant_t = pointer_variant_from_list_t<valid_types_t>::type;

    // define the std::variant of std::functions taking a const-ref of the types from valid_types_t
    using update_cbk_variant_t = function_variant_from_list_t<valid_types_t>::type;

    // | ------------------------- Members ------------------------ |
    rclcpp::Node& node; // non-owning reference (the memory is share-owned by the DynparamMgr)
    ParamProvider::resolved_name_t resolved_name;
    rclcpp::ParameterType type;
    param_ptr_variant_t param_ptr;
    update_cbk_variant_t update_cbk;
  
    // | ------------------------- Methods ------------------------ |
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
            RCLCPP_ERROR_STREAM_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "Cannot call update callback for parameter \"" << resolved_name.str << "\" - incompatible callback type!");
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
