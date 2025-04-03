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
    DynparamMgr(const std::shared_ptr<rclcpp::Node>& node, std::mutex& mtx, const std::string& node_name = std::string());

    rcl_interfaces::msg::SetParametersResult cbk_param_update(const std::vector<rclcpp::Parameter>& parameters);

    template <typename MemT>
    bool register_param(const std::string& name, MemT& param_var)
    {
      const bool success = m_pp.getParam(name, param_var, true);
      if (!success)
        return false;

      m_registered_params.emplace_back(
            name,
            std::any{std::reference_wrapper(param_var)}
          );
      return true;
    }

  private:
    std::shared_ptr<rclcpp::Node> m_node;
    mrs_lib::ParamProvider m_pp;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_cbk;

    struct registered_param_t
    {
      std::string name;
      std::any param_ref;
    };
    std::mutex& m_mtx;
    std::vector<registered_param_t> m_registered_params;
  };

}  // namespace mrs_lib

