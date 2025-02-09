#ifndef SERVICE_CLIENT_HANDLER_HPP
#define SERVICE_CLIENT_HANDLER_HPP

#ifndef SERVICE_CLIENT_HANDLER_H
#include <mrs_lib/service_client_handler.h>
#endif

namespace mrs_lib
{

// --------------------------------------------------------------
// |                  ServiceClientHandler_impl                 |
// --------------------------------------------------------------

/* ServiceClientHandler_impl(void) //{ */

template <class ServiceType>
ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(void) : service_initialized_(false) {
}

//}

/* ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address) //{ */

template <class ServiceType>
ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address) {


  node_ = node;

  {
    std::scoped_lock lock(mutex_service_client_);

    service_client_ = node->create_client<ServiceType>(address);
  }

  _address_ = address;

  service_initialized_ = true;
}

//}

/* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::shared_ptr<typename ServiceType::Response> ServiceClientHandler_impl<ServiceType>::callSync(
    const std::shared_ptr<typename ServiceType::Request>& request) {

  if (!service_initialized_) {
    return nullptr;
  }

  /* always check if the service is ready before calling */
  if (service_client_->service_is_ready()) {

    /* the future done callback is being run in a separate thread under the default callback group of the node */
    const auto future_msg = service_client_->async_send_request(request).future.share();

    /* it is a good practice to check if the future object is not already invalid after the call */
    /* if valid() is false, the future has UNDEFINED behavior */
    if (future_msg.valid() == false) {
      return nullptr;
    }

    return future_msg.get();

  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Can not call service in sync mode. Service not ready!");
    return nullptr;
  }

  return nullptr;
}

//}

/* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::shared_future<std::shared_ptr<typename ServiceType::Response>> ServiceClientHandler_impl<ServiceType>::callAsync(
    const std::shared_ptr<typename ServiceType::Request>& request) {

  std::shared_future<std::shared_ptr<typename ServiceType::Response>> future_response;

  future_response = service_client_->async_send_request(request).future.share();

  return future_response;
}

//}

// --------------------------------------------------------------
// |                    ServiceClientHandler                    |
// --------------------------------------------------------------

/* operator= //{ */

template <class ServiceType>
ServiceClientHandler<ServiceType>& ServiceClientHandler<ServiceType>::operator=(const ServiceClientHandler<ServiceType>& other) {

  if (this == &other) {
    return *this;
  }

  if (other.impl_) {
    this->impl_ = other.impl_;
  }

  return *this;
}

//}

/* copy constructor //{ */

template <class ServiceType>
ServiceClientHandler<ServiceType>::ServiceClientHandler(const ServiceClientHandler<ServiceType>& other) {

  if (other.impl_) {
    this->impl_ = other.impl_;
  }
}

//}

/* ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address) //{ */

template <class ServiceType>
ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address) {

  impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(node, address);
}

//}

/* initialize(rclcpp::Node::SharedPtr& node, const std::string& address) //{ */

template <class ServiceType>
void ServiceClientHandler<ServiceType>::initialize(rclcpp::Node::SharedPtr& node, const std::string& address) {

  impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(node, address);
}

//}

/* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::shared_ptr<typename ServiceType::Response> ServiceClientHandler<ServiceType>::callSync(const std::shared_ptr<typename ServiceType::Request>& request) {

  return impl_->callSync(request);
}

//}

/* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::shared_future<std::shared_ptr<typename ServiceType::Response>> ServiceClientHandler<ServiceType>::callAsync(
    const std::shared_ptr<typename ServiceType::Request>& request) {

  return impl_->callAsync(request);
}

//}

}  // namespace mrs_lib

#endif  // SERVICE_CLIENT_HANDLER_HPP
