// clang: MatousFormat
/**  \file
     \brief Implements DynparamMgr - a convenience class for managing dynamic ROS parameters.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#include <mrs_lib/dynamic_params.h>

namespace mrs_lib
{

  DynparamMgr::DynparamMgr(const std::shared_ptr<rclcpp::Node>& node, std::mutex& mtx, const std::string& node_name)
    : m_node(node), m_pp(node, node_name), m_mtx(mtx)
  {
    /* decltype(decltype(m_param_cbk)::element_type::callback) cbk = std::bind(&DynparamMgr::cbk_param_update, this, std::placeholders::_1); */
    /* m_param_cbk = m_node->add_on_set_parameters_callback(cbk); */

  }

  template <typename T>
  bool update_value(std::any& value_ref, const T& new_value)
  {
    try
    {
     T& cast_value = std::any_cast<T&>(value_ref);
     cast_value = new_value;
     return true;
    }
    catch (const std::bad_any_cast& e)
    {
      return false;
    }
  }

  rcl_interfaces::msg::SetParametersResult DynparamMgr::cbk_param_update(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    std::stringstream reason;

    std::scoped_lock lck(m_mtx);
    for (const auto& new_param : parameters)
    {
      // try to find the parameter by name
      const auto& param_name = new_param.get_name();
      const auto found_it = std::find_if(
            std::begin(m_registered_params), std::end(m_registered_params),
            [&param_name](const auto& registered_param)
            {
              return registered_param.name == param_name;
            }
          );

      // if the parameter was not found, ignore it
      if (found_it == std::end(m_registered_params))
        continue;

      // update the parameter value
      auto& found_el = *found_it;
      using type_enum = rclcpp::ParameterType;
      switch (new_param.get_type())
      {
        case type_enum::PARAMETER_BOOL:
          result.successful = update_value(found_el.param_ref, new_param.as_bool()); break;
        case type_enum::PARAMETER_INTEGER:
          result.successful = update_value(found_el.param_ref, new_param.as_int()); break;
        case type_enum::PARAMETER_DOUBLE:
          result.successful = update_value(found_el.param_ref, new_param.as_double()); break;
        case type_enum::PARAMETER_STRING:
          result.successful = update_value(found_el.param_ref, new_param.as_string()); break;
        case type_enum::PARAMETER_BOOL_ARRAY:
          result.successful = update_value(found_el.param_ref, new_param.as_bool_array()); break;
        case type_enum::PARAMETER_BYTE_ARRAY:
          result.successful = update_value(found_el.param_ref, new_param.as_byte_array()); break;
        case type_enum::PARAMETER_INTEGER_ARRAY:
          result.successful = update_value(found_el.param_ref, new_param.as_integer_array()); break;
        case type_enum::PARAMETER_DOUBLE_ARRAY:
          result.successful = update_value(found_el.param_ref, new_param.as_double_array()); break;
        case type_enum::PARAMETER_STRING_ARRAY:
          result.successful = update_value(found_el.param_ref, new_param.as_string_array()); break;
        case type_enum::PARAMETER_NOT_SET:
          result.successful = false; break;
      }

      if (!result.successful)
      {
        if (new_param.get_type() == type_enum::PARAMETER_NOT_SET)
          reason << "Parameter " << param_name << " is not set!\n";
        else
          reason << "Parameter " << param_name << " has a wrong type (" << new_param.get_type_name() << ")!\n";
      }
    }

    result.reason = reason.str();
    // remove the trailing \n if there is one
    if (!result.reason.empty())
      result.reason.pop_back();
    return result;
  }

}  // namespace mrs_lib

