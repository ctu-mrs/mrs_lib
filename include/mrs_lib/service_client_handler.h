/**  \file
     \brief Defines ServiceClientHandler and related convenience classes for upgrading the ROS service client
     \author Tomas Baca - tomas.baca@fel.cvut.cz
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <future>

namespace mrs_lib
{

  /* class ServiceClientHandler //{ */

  /**
   * @brief user wrapper of the service client handler implementation
   */
  template <class ServiceType>
  class ServiceClientHandler {

  public:

    /**
     * @brief The main constructor with all the options.
     *
     * For a more detailed explanation of the parameters, see the documentation of rclcpp::Node::create_client.
     *
     * @param node            ROS node handler
     * @param address         Name of the service
     * @param callback_group  Callback group used internally by the node for the response callback
     * @param qos QOS         Communication quality of service profile
     */
    ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos = rclcpp::ServicesQoS(), const rclcpp::CallbackGroup::SharedPtr& callback_group = nullptr);

    /*!
     * @brief Default constructor to avoid having to use pointers.
     *
     * It does nothing and the object it constructs will also do nothing.
     * Use some of the other constructors for a construction of an actually usable object.
     */
    ServiceClientHandler();

    /**
     * @brief A convenience constructor.
     *
     * This is just for convenience when you want to specify the callback group but don't care about QoS.
     *
     * @param node            ROS node handler
     * @param address         Name of the service
     * @param callback_group  Callback group used internally by the node for the response callback
     */
    ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group);

    /**
     * @brief synchronous call
     *
     * @param request The service request to be sent with the call
     *
     * @return Optional shared pointer to the result or std::nullopt if the call failed
     */
    std::optional<std::shared_ptr<typename ServiceType::Response>> callSync(const std::shared_ptr<typename ServiceType::Request>& request);

    /**
     * @brief asynchronous call
     *
     * @param request The service request to be sent with the call
     *
     * @return Optional shared pointer to the result or std::nullopt if the call failed
     */
    std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> callAsync(const std::shared_ptr<typename ServiceType::Request>& request);

  private:
    class Impl;
    std::shared_ptr<Impl> impl_;
  };

  //}

}  // namespace mrs_lib

#include <mrs_lib/impl/service_client_handler.hpp>
