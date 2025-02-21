/**  \file
     \brief Defines ServiceClientHandler and related convenience classes for upgrading the ROS service client
     \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef SERVICE_CLIENT_HANDLER_H
#define SERVICE_CLIENT_HANDLER_H

#include <rclcpp/rclcpp.hpp>

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
   * @param node ROS node handler
   * @param address service address
   */
  ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address);


  /**
   * @brief constructor
   *
   * @param node ROS node handler
   * @param address service address
   * @param qos QOS
   */
  ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos);

  /**
   * @brief constructor
   *
   * @param node ROS node handler
   * @param address service address
   * @param callback_group callback group
   */
  ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group);

  /**
   * @brief constructor
   *
   * @param node ROS node handler
   * @param address service address
   * @param qos QOS
   * @param callback_group callback group
   */
  ServiceClientHandler_impl(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos,
                            const rclcpp::CallbackGroup::SharedPtr& callback_group);

  /**
   * @brief "classic" synchronous service call
   *
   * @param request request
   *
   * @return optional shared pointer to the response
   */
  std::optional<std::shared_ptr<typename ServiceType::Response>> callSync(const std::shared_ptr<typename ServiceType::Request>& request);

  /**
   * @brief asynchronous service call
   *
   * @param request request
   *
   * @return optional shared future to the result
   */
  std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> callAsync(const std::shared_ptr<typename ServiceType::Request>& request);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::QoS                      qos_ = rclcpp::ServicesQoS();

  rclcpp::Client<ServiceType>::SharedPtr service_client_;
  std::mutex                             mutex_service_client_;
  std::atomic<bool>                      service_initialized_;

  std::string _address_;
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
   * @return itself
   */
  ServiceClientHandler& operator=(const ServiceClientHandler& other);

  /**
   * @brief copy constructor
   *
   * @param other
   */
  ServiceClientHandler(const ServiceClientHandler& other);

  /**
   * @brief basic constructor
   *
   * @param node ROS node handler
   * @param address service address
   */
  ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address);

  /**
   * @brief basic constructor
   *
   * @param node ROS node handler
   * @param address service address
   * @param qos QOS
   */
  ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos);

  /**
   * @brief basic constructor
   *
   * @param node ROS node handler
   * @param address service address
   * @param callback_group callback group
   */
  ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group);

  /**
   * @brief basic constructor
   *
   * @param node ROS node handler
   * @param address service address
   * @param callback_group callback group
   * @param qos QOS
   */
  ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos,
                       const rclcpp::CallbackGroup::SharedPtr& callback_group);

  /**
   * @brief synchronous call
   *
   * @param request request
   *
   * @return optional shared pointer to the result
   */
  std::optional<std::shared_ptr<typename ServiceType::Response>> callSync(const std::shared_ptr<typename ServiceType::Request>& request);

  /**
   * @brief asynchronous call
   *
   * @param request request
   *
   * @return optional shared future to the result
   */
  std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> callAsync(const std::shared_ptr<typename ServiceType::Request>& request);

private:
  std::shared_ptr<ServiceClientHandler_impl<ServiceType>> impl_;
};

//}

}  // namespace mrs_lib

#ifndef SERVICE_CLIENT_HANDLER_HPP
#include <mrs_lib/impl/service_client_handler.hpp>
#endif

#endif
