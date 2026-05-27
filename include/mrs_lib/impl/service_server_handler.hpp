/**  \file
     \brief Implements the ServiceServerHandler wrapper to ROS2's ServiceServer
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#pragma once

#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/coro/runners.hpp>
#include <mrs_lib/coro/task.hpp>
#include <mrs_lib/internal/coroutine_callback_helpers.hpp>

namespace mrs_lib
{

  // --------------------------------------------------------------
  // |                    ServiceServerHandler                    |
  // --------------------------------------------------------------

  /* ServiceServerHandler() constructors //{ */

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk,
                                                          const rclcpp::QoS& qos)
      : ServiceServerHandler(node, address, cbk, qos, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  {
  }

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler()
  {
  }

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk,
                                                          const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group)
      : callback_group_(callback_group), service_server_(node->create_service<ServiceType>(address, cbk, qos, callback_group))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Created service: " << service_server_->get_service_name());
  }

  template <class ServiceType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk,
                                                          const rclcpp::CallbackGroup::SharedPtr& callback_group)
      : ServiceServerHandler(node, address, cbk, rclcpp::ServicesQoS(), callback_group)
  {
  }

  template <class ServiceType>
  template <typename ClassType>
  ServiceServerHandler<ServiceType>::ServiceServerHandler(
      rclcpp::Node::SharedPtr& node, const std::string& address,
      mrs_lib::Task<bool> (ClassType::*method)(const std::shared_ptr<typename ServiceType::Request> request,
                                               const std::shared_ptr<typename ServiceType::Response> response),
      ClassType* instance, const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group)
      : callback_group_(callback_group)
  {
    internal::require_callback_group_coro_compatible(callback_group);

    auto is_running = std::make_shared<std::atomic<bool>>(false);

    // 1. Create a shared pointer to hold the service server.
    // This will survive moves and copies of the ServiceServerHandler.
    auto safe_server_ptr = std::make_shared<typename rclcpp::Service<ServiceType>::SharedPtr>();

    // 2. Capture 'safe_server_ptr' instead of 'this'
    auto deferred_cbk = [safe_server_ptr, is_running, method, instance](const std::shared_ptr<rmw_request_id_t> req_id,
                                                                        const std::shared_ptr<typename ServiceType::Request> req) -> void {
      bool was_running = is_running->exchange(true);

      if (!was_running)
      {
        coro::internal::start_task(
            [](std::shared_ptr<typename rclcpp::Service<ServiceType>::SharedPtr> server, // Pass the captured safe pointer
               std::shared_ptr<std::atomic<bool>> is_running,
               mrs_lib::Task<bool> (ClassType::*method)(const std::shared_ptr<typename ServiceType::Request>,
                                                        const std::shared_ptr<typename ServiceType::Response>),
               ClassType* instance, std::shared_ptr<rmw_request_id_t> req_id, std::shared_ptr<typename ServiceType::Request> req) -> mrs_lib::Task<void> {
              auto res = std::make_shared<typename ServiceType::Response>();

              co_await std::invoke(method, instance, req, res);

              // 3. Dereference the safe pointer to get the actual service object and send the response
              if (*server)
              {
                (*server)->send_response(*req_id, *res);
              }

              is_running->store(false);
            },
            safe_server_ptr, is_running, method, instance, req_id, req); // Pass it into the coroutine
      }
    };

    // 4. Create the service and store it in both the class member AND the shared pointer we captured
    service_server_ = node->create_service<ServiceType>(address, deferred_cbk, qos, callback_group);
    *safe_server_ptr = service_server_;
  }

  //}

} // namespace mrs_lib
