#ifndef SERVICE_CLIENT_HANDLER_HPP
#define SERVICE_CLIENT_HANDLER_HPP

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

/* ServiceClientHandler_impl(ros::NodeHandle& nh, const std::string& address) //{ */

template <class ServiceType>
ServiceClientHandler_impl<ServiceType>::ServiceClientHandler_impl(ros::NodeHandle& nh, const std::string& address) {

  {
    std::scoped_lock lock(mutex_service_client_);

    service_client_ = nh.serviceClient<ServiceType>(address);
  }

  _address_       = address;
  async_attempts_ = 1;

  /* thread_oneshot_ = std::make_shared<std::thread>(std::thread(&ServiceClientHandler_impl::threadOneshot, this, true, false)); */

  service_initialized_ = true;
}

//}

/* call(ServiceType& srv) //{ */

template <class ServiceType>
bool ServiceClientHandler_impl<ServiceType>::call(ServiceType& srv) {

  if (!service_initialized_) {
    return false;
  }

  return service_client_.call(srv);
}

//}

/* call(ServiceType& srv, const int& attempts) //{ */

template <class ServiceType>
bool ServiceClientHandler_impl<ServiceType>::call(ServiceType& srv, const int& attempts) {

  if (!service_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_service_client_);

  bool success = false;
  int  counter = 0;

  while (!success && ros::ok()) {

    success = service_client_.call(srv);

    if (!success) {
      ROS_ERROR("[%s]: failed to call service to '%s'", ros::this_node::getName().c_str(), _address_.c_str());
    }

    if (++counter >= attempts) {
      break;
    }
  }

  return success;
}

//}

/* call(ServiceType& srv, const int& attempts, const double& repeat_delay) //{ */

template <class ServiceType>
bool ServiceClientHandler_impl<ServiceType>::call(ServiceType& srv, const int& attempts, const double& repeat_delay) {

  if (!service_initialized_) {
    return false;
  }

  std::scoped_lock lock(mutex_service_client_);

  bool success = false;
  int  counter = 0;

  while (!success && ros::ok()) {

    success = service_client_.call(srv);

    if (!success) {
      ROS_ERROR("[%s]: failed to call service to '%s'", ros::this_node::getName().c_str(), _address_.c_str());
    }

    if (++counter >= attempts) {
      break;
    }

    ros::Duration(repeat_delay).sleep();
  }

  return success;
}

//}

/* callAsync(ServiceType& srv) //{ */

template <class ServiceType>
std::future<ServiceType> ServiceClientHandler_impl<ServiceType>::callAsync(ServiceType& srv) {

  {
    std::scoped_lock lock(mutex_async_);

    async_data_         = srv;
    async_attempts_     = 1;
    async_repeat_delay_ = 0;
  }

  return std::async(std::launch::async, &ServiceClientHandler_impl::asyncRun, this);
}

//}

/* callAsync(ServiceType& srv, const int& attempts) //{ */

template <class ServiceType>
std::future<ServiceType> ServiceClientHandler_impl<ServiceType>::callAsync(ServiceType& srv, const int& attempts) {

  {
    std::scoped_lock lock(mutex_async_);

    async_data_         = srv;
    async_attempts_     = attempts;
    async_repeat_delay_ = 0;
  }

  return std::async(std::launch::async, &ServiceClientHandler_impl::asyncRun, this);
}

//}

/* callAsync(ServiceType& srv, const int& attempts, const double &repeat_delay) //{ */

template <class ServiceType>
std::future<ServiceType> ServiceClientHandler_impl<ServiceType>::callAsync(ServiceType& srv, const int& attempts, const double& repeat_delay) {

  {
    std::scoped_lock lock(mutex_async_);

    async_data_         = srv;
    async_attempts_     = attempts;
    async_repeat_delay_ = repeat_delay;
  }

  return std::async(std::launch::async, &ServiceClientHandler_impl::asyncRun, this);
}

//}

/* asyncRun(void) //{ */

template <class ServiceType>
ServiceType ServiceClientHandler_impl<ServiceType>::asyncRun(void) {

  ServiceType async_data;
  int         async_attempts;

  {
    std::scoped_lock lock(mutex_async_);

    async_data          = async_data_;
    async_attempts      = async_attempts_;
    async_repeat_delay_ = async_repeat_delay_;
  }

  call(async_data, async_attempts, async_repeat_delay_);

  return async_data;
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

/* ServiceClientHandler(ros::NodeHandle& nh, const std::string& address) //{ */

template <class ServiceType>
ServiceClientHandler<ServiceType>::ServiceClientHandler(ros::NodeHandle& nh, const std::string& address) {

  impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(nh, address);
}

//}

/* initialize(ros::NodeHandle& nh, const std::string& address) //{ */

template <class ServiceType>
void ServiceClientHandler<ServiceType>::initialize(ros::NodeHandle& nh, const std::string& address) {

  impl_ = std::make_shared<ServiceClientHandler_impl<ServiceType>>(nh, address);
}

//}

/* call(ServiceType& srv) //{ */

template <class ServiceType>
bool ServiceClientHandler<ServiceType>::call(ServiceType& srv) {

  return impl_->call(srv);
}

//}

/* call(ServiceType& srv, const int& attempts) //{ */

template <class ServiceType>
bool ServiceClientHandler<ServiceType>::call(ServiceType& srv, const int& attempts) {

  return impl_->call(srv, attempts);
}

//}

/* call(ServiceType& srv, const int& attempts, const double& repeat_delay) //{ */

template <class ServiceType>
bool ServiceClientHandler<ServiceType>::call(ServiceType& srv, const int& attempts, const double& repeat_delay) {

  return impl_->call(srv, attempts, repeat_delay);
}

//}

/* callAsync(ServiceType& srv) //{ */

template <class ServiceType>
std::future<ServiceType> ServiceClientHandler<ServiceType>::callAsync(ServiceType& srv) {

  std::future<ServiceType> res = impl_->callAsync(srv);

  return res;
}

//}

/* callAsync(ServiceType& srv, const int& attempts) //{ */

template <class ServiceType>
std::future<ServiceType> ServiceClientHandler<ServiceType>::callAsync(ServiceType& srv, const int& attempts) {

  std::future<ServiceType> res = impl_->callAsync(srv, attempts);

  return res;
}

//}

/* callAsync(ServiceType& srv, const int& attempts, const double& repeat_delay) //{ */

template <class ServiceType>
std::future<ServiceType> ServiceClientHandler<ServiceType>::callAsync(ServiceType& srv, const int& attempts, const double& repeat_delay) {

  std::future<ServiceType> res = impl_->callAsync(srv, attempts, repeat_delay);

  return res;
}

//}

}  // namespace mrs_lib

#endif  // SERVICE_CLIENT_HANDLER_HPP
