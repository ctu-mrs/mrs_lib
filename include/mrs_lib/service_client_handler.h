/**  \file
     \brief Defines ServiceClientHandler and related convenience classes for upgrading the ROS service client
     \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef SERVICE_CLIENT_HANDLER_H
#define SERVICE_CLIENT_HANDLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <future>
#include <any>
#include <mutex>

namespace mrs_lib
{

/* class ServiceClientHandler //{ */

/**
 * @brief user wrapper of the service client handler implementation
 */
class ServiceClientHandler {

public:
  ServiceClientHandler() = default;

  /**
   * @brief initializer
   *
   * @param nh ROS node handler
   * @param address service address
   */
  template <class ServiceType>
  void initialize(ros::NodeHandle& nh, const std::string& address);

  /**
   * @brief "standard" synchronous call
   *
   * @param srv data
   *
   * @return true when success
   */
  template <class ServiceType>
  bool call(ServiceType& srv);

private:
  ros::ServiceClient service_client_;
};

//}

}  // namespace mrs_lib

#include <impl/service_client_handler.hpp>

#endif  // SERVICE_CLIENT_HANDLER_H
