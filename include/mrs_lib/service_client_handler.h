#ifndef SERVICE_CLIENT_HANDLER_H
#define SERVICE_CLIENT_HANDLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <mutex>
#include <future>

#include <mrs_lib/mutex.h>

#include <std_srvs/Trigger.h>

namespace mrs_lib
{

/* class ServiceClientHandler_impl //{ */

template <class ServiceType>
class ServiceClientHandler_impl {

public:
  ServiceClientHandler_impl(void);

  ~ServiceClientHandler_impl(void){};

  ServiceClientHandler_impl(ros::NodeHandle& nh, const std::string& address);

  bool call(void);

  bool call(ServiceType& srv);

  bool call(ServiceType& srv, const int& attempts);

  std::future<ServiceType> callAsync(ServiceType& srv);

  std::future<ServiceType> callAsync(ServiceType& srv, const int& attempts);

private:
  ros::ServiceClient service_client_;
  std::mutex         mutex_service_client_;
  std::atomic<bool>  service_initialized_;

  std::string _address_;

  ServiceType async_data_;
  int         async_attempts_;
  std::mutex  mutex_async_;

  ServiceType asyncRun(void);
};

//}

/* class ServiceClientHandler //{ */

template <class ServiceType>
class ServiceClientHandler {

public:
  ServiceClientHandler(){};
  ~ServiceClientHandler(){};

  ServiceClientHandler& operator=(const ServiceClientHandler& other);

  ServiceClientHandler(const ServiceClientHandler& other);

  ServiceClientHandler(ros::NodeHandle& nh, const std::string& address);

  bool call(ServiceType& srv);

  bool call(ServiceType& srv, const int& attempts);

  std::future<ServiceType> callAsync(ServiceType& srv);

  std::future<ServiceType> callAsync(ServiceType& srv, const int& attempts);

private:
  std::shared_ptr<ServiceClientHandler_impl<ServiceType>> impl_;
};

//}

}  // namespace mrs_lib

#include <impl/service_client_handler.hpp>

#endif  // SERVICE_CLIENT_HANDLER_H
