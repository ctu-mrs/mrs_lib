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
  ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(void) : service_initialized_(false)
  {
  }

  //}

  /* ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address) //{ */

  template <class ServiceType>
  ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address)
  {


    node_ = node;

    {
      std::scoped_lock lock(mutex_service_client_);

      service_client_ = node->create_client<ServiceType>(address);
    }

    _address_ = address;
    async_attempts_ = 1;

    service_initialized_ = true;
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  bool ServiceClientHandler_impl<ServiceType>::callSync(const ServiceType::Request& request, ServiceType::Response& response)
  {

    if (!service_initialized_)
    {
      return false;
    }

    /* always check if the service is ready before calling */
    if (service_client_->service_is_ready())
    {

      std::shared_ptr<typename ServiceType::Request> request_ptr = std::make_shared<typename ServiceType::Request>(request);

      /* the future done callback is being run in a separate thread under the default callback group of the node */
      const auto future_msg = service_client_->async_send_request(request_ptr).future.share();

      /* it is a good practice to check if the future object is not already invalid after the call */
      /* if valid() is false, the future has UNDEFINED behavior */
      if (future_msg.valid() == false)
      {
        return false;
      }

      response = *future_msg.get();

    } else
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Can not call service in sync mode. Service not ready!");
      return false;
    }

    return true;
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts) //{ */

  template <class ServiceType>
  bool ServiceClientHandler_impl<ServiceType>::callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts)
  {

    if (!service_initialized_)
    {
      return false;
    }

    std::scoped_lock lock(mutex_service_client_);

    bool success = false;
    int counter = 0;

    while (!success && rclcpp::ok())
    {

      /* always check if the service is ready before calling */
      if (service_client_->service_is_ready())
      {

        typename ServiceType::Request::SharedPtr request_ptr(request);

        /* the future done callback is being run in a separate thread under the default callback group of the node */
        const auto future_msg = service_client_->async_send_request(request_ptr).future.share();

        /* it is a good practice to check if the future object is not already invalid after the call */
        /* if valid() is false, the future has UNDEFINED behavior */
        if (future_msg.valid() == false)
        {
          success = false;
        } else
        {

          response = future_msg.get();
          success = true;
        }

      } else
      {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Can not call service in sync mode. Service not ready!");
      }

      if (!success)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service to '%s'", _address_.c_str());
      }

      if (++counter >= attempts)
      {
        break;
      }
    }

    return success;
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts, const double& repeat_delay) //{ */

  template <class ServiceType>
  bool ServiceClientHandler_impl<ServiceType>::callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts,
                                                        const double& repeat_delay)
  {

    if (!service_initialized_)
    {
      return false;
    }

    std::scoped_lock lock(mutex_service_client_);

    bool success = false;
    int counter = 0;

    while (!success && rclcpp::ok())
    {

      /* always check if the service is ready before calling */
      if (service_client_->service_is_ready())
      {

        typename ServiceType::Request::SharedPtr request_ptr(request);

        /* the future done callback is being run in a separate thread under the default callback group of the node */
        const auto future_msg = service_client_->async_send_request(request_ptr).future.share();

        /* it is a good practice to check if the future object is not already invalid after the call */
        /* if valid() is false, the future has UNDEFINED behavior */
        if (future_msg.valid() == false)
        {
          success = false;
        } else
        {

          response = future_msg.get();
          success = true;
        }

      } else
      {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Can not call service in sync mode. Service not ready!");
      }

      if (!success)
      {
        RCLCPP_ERROR(node_->get_logger(), "failed to call service to '%s'", _address_.c_str());
      }

      if (++counter >= attempts)
      {
        break;
      }

      node_->get_clock()->sleep_for(std::chrono::duration<double>(repeat_delay));
    }

    return success;
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  std::future<ServiceType> ServiceClientHandler_impl<ServiceType>::callAsync(const ServiceType::Request& request, ServiceType::Response& response)
  {

    /* { */
    /*   std::scoped_lock lock(mutex_async_); */

    /*   async_data_ = srv; */
    /*   async_attempts_ = 1; */
    /*   async_repeat_delay_ = 0; */
    /* } */

    /* return std::async(std::launch::async, &ServiceClientHandler_impl::asyncRun, this); */
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts) //{ */

  template <class ServiceType>
  std::future<ServiceType> ServiceClientHandler_impl<ServiceType>::callAsync(const ServiceType::Request& request, ServiceType::Response& response,
                                                                             const int& attempts)
  {

    /* { */
    /*   std::scoped_lock lock(mutex_async_); */

    /*   async_data_ = srv; */
    /*   async_attempts_ = attempts; */
    /*   async_repeat_delay_ = 0; */
    /* } */

    /* return std::async(std::launch::async, &ServiceClientHandler_impl::asyncRun, this); */
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts, const double &repeat_delay) //{ */

  template <class ServiceType>
  std::future<ServiceType> ServiceClientHandler_impl<ServiceType>::callAsync(const ServiceType::Request& request, ServiceType::Response& response,
                                                                             const int& attempts, const double& repeat_delay)
  {

    /* { */
    /*   std::scoped_lock lock(mutex_async_); */

    /*   async_data_ = srv; */
    /*   async_attempts_ = attempts; */
    /*   async_repeat_delay_ = repeat_delay; */
    /* } */

    /* return std::async(std::launch::async, &ServiceClientHandler_impl::asyncRun, this); */
  }

  //}

  /* asyncRun(void) //{ */

  template <class ServiceType>
  ServiceType ServiceClientHandler_impl<ServiceType>::asyncRun(void)
  {

    ServiceType async_data;
    int async_attempts;
    double async_repeat_delay;

    {
      std::scoped_lock lock(mutex_async_);

      async_data = async_data_;
      async_attempts = async_attempts_;
      async_repeat_delay = async_repeat_delay_;
    }

    callSync(async_data, async_attempts, async_repeat_delay);

    return async_data;
  }

  //}

  // --------------------------------------------------------------
  // |                    ServiceClientHandler                    |
  // --------------------------------------------------------------

  /* operator= //{ */

  template <class ServiceType>
  ServiceClientHandler<ServiceType>& ServiceClientHandler<ServiceType>::operator=(const ServiceClientHandler<ServiceType>& other)
  {

    if (this == &other)
    {
      return *this;
    }

    if (other.impl_)
    {
      this->impl_ = other.impl_;
    }

    return *this;
  }

  //}

  /* copy constructor //{ */

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(const ServiceClientHandler<ServiceType>& other)
  {

    if (other.impl_)
    {
      this->impl_ = other.impl_;
    }
  }

  //}

  /* ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address) //{ */

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address)
  {

    impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(node, address);
  }

  //}

  /* initialize(rclcpp::Node::SharedPtr& node, const std::string& address) //{ */

  template <class ServiceType>
  void ServiceClientHandler<ServiceType>::initialize(rclcpp::Node::SharedPtr& node, const std::string& address)
  {

    impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(node, address);
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  bool ServiceClientHandler<ServiceType>::callSync(const ServiceType::Request& request, ServiceType::Response& response)
  {

    return impl_->callSync(request, response);
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts) //{ */

  template <class ServiceType>
  bool ServiceClientHandler<ServiceType>::callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts)
  {

    return impl_->callSync(request, response, attempts);
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts, const double& repeat_delay) //{ */

  template <class ServiceType>
  bool ServiceClientHandler<ServiceType>::callSync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts,
                                                   const double& repeat_delay)
  {

    return impl_->callSync(request, response, attempts, repeat_delay);
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  std::future<ServiceType> ServiceClientHandler<ServiceType>::callAsync(const ServiceType::Request& request, ServiceType::Response& response)
  {

    std::future<ServiceType> res = impl_->callAsync(request, response);

    return res;
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts) //{ */

  template <class ServiceType>
  std::future<ServiceType> ServiceClientHandler<ServiceType>::callAsync(const ServiceType::Request& request, ServiceType::Response& response,
                                                                        const int& attempts)
  {

    std::future<ServiceType> res = impl_->callAsync(request, response, attempts);

    return res;
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response, const int& attempts, const double& repeat_delay) //{ */

  template <class ServiceType>
  std::future<ServiceType> ServiceClientHandler<ServiceType>::callAsync(const ServiceType::Request& request, ServiceType::Response& response,
                                                                        const int& attempts, const double& repeat_delay)
  {

    std::future<ServiceType> res = impl_->callAsync(request, response, attempts, repeat_delay);

    return res;
  }

  //}

}  // namespace mrs_lib

#endif  // SERVICE_CLIENT_HANDLER_HPP
