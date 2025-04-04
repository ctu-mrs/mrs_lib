/**  \file
     \brief Implements the ServiceServerHandler wrapper to ROS2's ServiceServer
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#pragma once

#include <mrs_lib/service_server_handler.h>

namespace mrs_lib
{

  // --------------------------------------------------------------
  // |                    ServiceServerHandler                    |
  // --------------------------------------------------------------

  /* ServiceServerHandler() constructors //{ */

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk, const rclcpp::QoS& qos)
    : ServiceServerHandler(node, address, cbk, qos, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  {
  }

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler()
  {
  }

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk, const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group)
    : callback_group_(callback_group),
      service_server_(node->create_service<ServiceType>(address, cbk, qos, callback_group))
  {
  }

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk, const rclcpp::CallbackGroup::SharedPtr& callback_group)
    : ServiceServerHandler(node, address, cbk, rclcpp::ServicesQoS(), callback_group)
  {
  }

  //}

}  // namespace mrs_lib
