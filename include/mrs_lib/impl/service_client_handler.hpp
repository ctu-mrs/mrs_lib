/**  \file
     \brief Implements ServiceClientHandler and related convenience classes for upgrading the ROS service client
     \author Tomas Baca - tomas.baca@fel.cvut.cz
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#pragma once

#include <mrs_lib/service_client_handler.h>

namespace mrs_lib
{

  // --------------------------------------------------------------
  // |                    ServiceClientHandler                    |
  // --------------------------------------------------------------

  /* ServiceClientHandler() constructors //{ */

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos)
    : impl_(std::make_shared<Impl>(node, address, qos, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)))
  {
  }

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler()
    : impl_(nullptr)
  {
  }

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group)
    : impl_(std::make_shared<Impl>(node, address, qos, callback_group))
  {
  }

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group)
    : ServiceClientHandler(node, address, rclcpp::ServicesQoS(), callback_group)
  {
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  std::optional<std::shared_ptr<typename ServiceType::Response>> ServiceClientHandler<ServiceType>::callSync(const std::shared_ptr<typename ServiceType::Request>& request)
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use callSync()!");
      return std::nullopt;
    }
    return impl_->callSync(request);
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> ServiceClientHandler<ServiceType>::callAsync(const std::shared_ptr<typename ServiceType::Request>& request)
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use callSync()!");
      return std::nullopt;
    }
    return impl_->callAsync(request);
  }

  //}

  // --------------------------------------------------------------
  // |                 ServiceClientHandler::Impl                 |
  // --------------------------------------------------------------

  /* class ServiceClientHandler::impl //{ */

  /**
   * @brief implementation of the service client handler
   */
  template <class ServiceType>
  class ServiceClientHandler<ServiceType>::Impl {

  public:

    /**
     * @brief constructor
     *
     * @param node ROS node handler
     * @param address service address
     * @param qos QOS
     * @param callback_group callback group
     */
    Impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group)
      : callback_group_(callback_group),
        service_client_(node->create_client<ServiceType>(address, qos, callback_group))
    {
    }

    /**
     * @brief "classic" synchronous service call
     *
     * @param request request
     *
     * @return optional shared pointer to the response
     */
    std::optional<std::shared_ptr<typename ServiceType::Response>> callSync(const std::shared_ptr<typename ServiceType::Request>& request)
    {
      /* always check if the service is ready before calling */
      if (!service_client_->service_is_ready())
        return std::nullopt;

      /* the future done callback is being run in a separate thread under the default callback group of the node */
      const auto future_msg = service_client_->async_send_request(request).future.share();

      /* it is a good practice to check if the future object is not already invalid after the call */
      /* if valid() is false, the future has UNDEFINED behavior */
      if (!future_msg.valid())
        return std::nullopt;

      // wait for the future to become available and then return
      return future_msg.get();
    }

    /**
     * @brief asynchronous service call
     *
     * @param request request
     *
     * @return optional shared future to the result
     */
    std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> callAsync(const std::shared_ptr<typename ServiceType::Request>& request)
    {
      /* always check if the service is ready before calling */
      if (!service_client_->service_is_ready())
        return std::nullopt;

      const auto future = service_client_->async_send_request(request).future.share();

      /* it is a good practice to check if the future object is not already invalid after the call */
      /* if valid() is false, the future has UNDEFINED behavior */
      if (!future.valid())
        return std::nullopt;

      return future;
    }

  private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    typename rclcpp::Client<ServiceType>::SharedPtr service_client_;
  };

  //}

}  // namespace mrs_lib
