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
#include <mutex>

namespace mrs_lib
{

/* class ServiceClientHandler_impl //{ */

/**
 * @brief implementation of the service client handler
 */
template <class ServiceType>
class ServiceClientHandler_impl {

public:
  /**
   * @brief default constructor
   */
  ServiceClientHandler_impl(void);

  /**
   * @brief default destructor
   */
  ~ServiceClientHandler_impl(void){};

  /**
   * @brief constructor
   *
   * @param nh ROS node handler
   * @param address service address
   */
  ServiceClientHandler_impl(ros::NodeHandle& nh, const std::string& address);

  /**
   * @brief "classic" synchronous service call
   *
   * @param srv data
   *
   * @return true when success
   */
  bool call(ServiceType& srv);

  /**
   * @brief "classic" synchronous service call with repeats after an error
   *
   * @param srv data
   * @param attempts how many attempts for the call
   *
   * @return  true when success
   */
  bool call(ServiceType& srv, const int& attempts);

  /**
   * @brief "classic" synchronous service call with repeats after an error
   *
   * @param srv data
   * @param attempts how many attempts for the call
   * @param repeat_delay how long to wait before repeating the call
   *
   * @return  true when success
   */
  bool call(ServiceType& srv, const int& attempts, const double& repeat_delay);

  /**
   * @brief asynchronous service call
   *
   * @param srv data
   *
   * @return future result
   */
  std::future<ServiceType> callAsync(ServiceType& srv);

  /**
   * @brief asynchronous service call with repeates after an error
   *
   * @param srv data
   * @param attempts how many attempts for the call
   *
   * @return future result
   */
  std::future<ServiceType> callAsync(ServiceType& srv, const int& attempts);

  /**
   * @brief asynchronous service call with repeates after an error
   *
   * @param srv data
   * @param attempts how many attempts for the call
   * @param repeat_delay how long to wait before repeating the call
   *
   * @return future result
   */
  std::future<ServiceType> callAsync(ServiceType& srv, const int& attempts, const double& repeat_delay);

private:
  ros::ServiceClient service_client_;
  std::mutex         mutex_service_client_;
  std::atomic<bool>  service_initialized_;

  std::string _address_;

  ServiceType async_data_;
  int         async_attempts_;
  double      async_repeat_delay_;
  std::mutex  mutex_async_;

  ServiceType asyncRun(void);
};

//}

/* class ServiceClientHandler //{ */

/**
 * @brief user wrapper of the service client handler implementation
 */
template <class ServiceType>
class ServiceClientHandler {

public:
  /**
   * @brief generic constructor
   */
  ServiceClientHandler(void){};

  /**
   * @brief generic destructor
   */
  ~ServiceClientHandler(void){};

  /**
   * @brief operator=
   *
   * @param other
   *
   * @return
   */
  ServiceClientHandler& operator=(const ServiceClientHandler& other);

  /**
   * @brief copy constructor
   *
   * @param other
   */
  ServiceClientHandler(const ServiceClientHandler& other);

  /**
   * @brief constructor
   *
   * @param nh ROS node handler
   * @param address service address
   */
  ServiceClientHandler(ros::NodeHandle& nh, const std::string& address);

  /**
   * @brief initializer
   *
   * @param nh ROS node handler
   * @param address service address
   */
  void initialize(ros::NodeHandle& nh, const std::string& address);

  /**
   * @brief "standard" synchronous call
   *
   * @param srv data
   *
   * @return true when success
   */
  bool call(ServiceType& srv);

  /**
   * @brief "standard" synchronous call with repeats after failure
   *
   * @param srv data
   * @param attempts how many attempts for the call
   *
   * @return true when success
   */
  bool call(ServiceType& srv, const int& attempts);

  /**
   * @brief "standard" synchronous call with repeats after failure
   *
   * @param srv data
   * @param attempts how many attempts for the call
   * @param repeat_delay how long to wait before repeating the call
   *
   * @return true when success
   */
  bool call(ServiceType& srv, const int& attempts, const double& repeat_delay);

  /**
   * @brief asynchronous call
   *
   * @param srv data
   *
   * @return future result
   */
  std::future<ServiceType> callAsync(ServiceType& srv);

  /**
   * @brief asynchronous call with repeats after failure
   *
   * @param srv data
   * @param attempts how many attemps for the call
   *
   * @return future result
   */
  std::future<ServiceType> callAsync(ServiceType& srv, const int& attempts);

  /**
   * @brief asynchronous call with repeats after failure
   *
   * @param srv data
   * @param attempts how many attemps for the call
   * @param repeat_delay how long to wait before repeating the call
   *
   * @return future result
   */
  std::future<ServiceType> callAsync(ServiceType& srv, const int& attempts, const double& repeat_delay);

private:
  std::shared_ptr<ServiceClientHandler_impl<ServiceType>> impl_;
};

//}

}  // namespace mrs_lib

#include <impl/service_client_handler.hpp>

#endif  // SERVICE_CLIENT_HANDLER_H
