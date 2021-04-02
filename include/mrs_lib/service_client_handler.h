#ifndef SERVICE_CLIENT_HANDLER_H
#define SERVICE_CLIENT_HANDLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <mutex>

#include <std_srvs/Trigger.h>

namespace mrs_lib
{

template <class ServiceType>
class ServiceClientHandler {

public:
  ServiceClientHandler() : service_initialized_(false) {
  }

  ~ServiceClientHandler() {};

  ServiceClientHandler(ros::NodeHandle& nh, const std::string& topic_name) {

    {
      std::scoped_lock lock(mutex_service_client_);

      service_client_ = std::make_shared<ros::ServiceClient>(nh.serviceClient<ServiceType>(topic_name));
    }

    service_initialized_ = true;
  }

  bool call(ServiceType& srv) {

    return service_client_->call(srv);
  }

  ServiceClientHandler(const ServiceClientHandler& other) {

    if (other.service_client_) {
      this->service_client_ = other.service_client_;
    }
  }

  ServiceClientHandler& operator=(const ServiceClientHandler& other) {

    if (this == &other) {
      return *this;
    }

    if (other.service_client_) {
      this->service_client_ = other.service_client_;
    }

    return *this;
  }

private:
  std::shared_ptr<ros::ServiceClient> service_client_;
  std::mutex                          mutex_service_client_;
  std::atomic<bool>                   service_initialized_;
};

}  // namespace mrs_lib

#include <impl/service_client_handler.hpp>

#endif  // SERVICE_CLIENT_HANDLER_H
