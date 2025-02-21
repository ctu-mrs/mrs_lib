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

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  {
    std::scoped_lock lock(mutex_service_client_);

    service_client_ = node->create_client<ServiceType>(address, qos_, callback_group_);
  }

  _address_ = address;

  service_initialized_ = true;
}

//}

/* ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos) //{ */

template <class ServiceType>
ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos) {

  qos_ = qos;

  ServiceClientHandler_impl(node, address);
}

//}

/* ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group) //{ */

template <class ServiceType>
ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address,
                                                                  const rclcpp::CallbackGroup::SharedPtr& callback_group) {

  callback_group_ = callback_group;

  ServiceClientHandler_impl(node, address);
}

//}

/* ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group) //{ */

template <class ServiceType>
ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos,
                                                                  const rclcpp::CallbackGroup::SharedPtr& callback_group) {

  qos_            = qos;
  callback_group_ = callback_group;

  ServiceClientHandler_impl(node, address);
}

//}

/* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::optional<std::shared_ptr<typename ServiceType::Response>> ServiceClientHandler_impl<ServiceType>::callSync(
    const std::shared_ptr<typename ServiceType::Request>& request) {

  if (!service_initialized_) {
    return {};
  }

  /* always check if the service is ready before calling */
  if (service_client_->service_is_ready()) {

    /* the future done callback is being run in a separate thread under the default callback group of the node */
    const auto future_msg = service_client_->async_send_request(request).future.share();

    /* it is a good practice to check if the future object is not already invalid after the call */
    /* if valid() is false, the future has UNDEFINED behavior */
    if (future_msg.valid() == false) {
      return {};
    }

    return future_msg.get();

  } else {
    // this branch occurs when the service can not contact the service server.
    return {};
  }

  return {};
}

//}

/* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> ServiceClientHandler_impl<ServiceType>::callAsync(
    const std::shared_ptr<typename ServiceType::Request>& request) {

  /* always check if the service is ready before calling */
  if (service_client_->service_is_ready()) {

    auto future = service_client_->async_send_request(request).future.share();

    /* it is a good practice to check if the future object is not already invalid after the call */
    /* if valid() is false, the future has UNDEFINED behavior */
    if (future.valid() == false) {
      return {};
    }

    return {future};

  } else {
    // this branch occurs when the service can not contact the service server.
    return {};
  }
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

/* ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos) //{ */

template <class ServiceType>
ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos) {

  impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(node, address, qos);
}

//}

/* ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group) //{ */

template <class ServiceType>
ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address,
                                                        const rclcpp::CallbackGroup::SharedPtr& callback_group) {

  impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(node, address, callback_group);
}

//}

/* ServiceClientHandler(node, address, qos, callback_group) //{ */

template <class ServiceType>
ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos,
                                                        const rclcpp::CallbackGroup::SharedPtr& callback_group) {

  impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(node, address, qos, callback_group);
}

//}

/* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::optional<std::shared_ptr<typename ServiceType::Response>> ServiceClientHandler<ServiceType>::callSync(
    const std::shared_ptr<typename ServiceType::Request>& request) {

  return impl_->callSync(request);
}

//}

/* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

template <class ServiceType>
std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> ServiceClientHandler<ServiceType>::callAsync(
    const std::shared_ptr<typename ServiceType::Request>& request) {

  return impl_->callAsync(request);
}

//}

}  // namespace mrs_lib

#endif  // SERVICE_CLIENT_HANDLER_HPP
