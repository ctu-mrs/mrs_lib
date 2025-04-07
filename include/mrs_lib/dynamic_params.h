// clang: MatousFormat
/**  \file
     \brief Defines DynparamMgr - a convenience class for managing dynamic ROS parameters.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/param_provider.h>
#include <any>

namespace mrs_lib
{

  class DynparamMgr
  {

  public:
    using param_type_variant_t = std::variant<
        int*,
        int64_t*,
        float*,
        double*,
        bool*,
        std::string*,
        std::vector<uint8_t>*,
        std::vector<int64_t>*,
        std::vector<double>*,
        std::vector<bool>*,
        std::vector<std::string>*
        >;

    DynparamMgr(const std::shared_ptr<rclcpp::Node>& node, std::mutex& mtx, const std::string& node_name = std::string());

    rcl_interfaces::msg::SetParametersResult update_to_ros();

    template <typename MemT>
    bool register_param(const std::string& name, MemT* param_var)
    {
      const bool success = m_pp.getParam(name, *param_var, true);
      if (!success)
        return false;

      m_registered_params.emplace_back(
            name,
            to_param_type<MemT>(),
            param_var
          );
      return true;
    }

  private:
    std::shared_ptr<rclcpp::Node> m_node;
    mrs_lib::ParamProvider m_pp;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_cbk;

    rcl_interfaces::msg::SetParametersResult cbk_param_update(const std::vector<rclcpp::Parameter>& parameters);

    struct registered_param_t
    {
      std::string name;
      rclcpp::ParameterType type;
      param_type_variant_t param_ptr;

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

    std::mutex& m_mtx;
    std::vector<registered_param_t> m_registered_params;
  };

}  // namespace mrs_lib

