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
     * This variant initializes a new MutuallyExclusive callback group for the service client, which is the
     * intended default behavior to avoid deadlocks when using the callSync() method.
     * For a more detailed explanation of the parameters, see the documentation of rclcpp::Node::create_client.
     *
     * @param node            ROS node handler.
     * @param address         Name of the service.
     * @param qos QOS         Communication quality of service profile.
     */
    ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos = rclcpp::ServicesQoS());

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
     * This is just for convenience when you want to specify the callback group.
     *
     * @param node            ROS node handler.
     * @param address         Name of the service.
     * @param qos QOS         Communication quality of service profile.
     * @param callback_group  Callback group used internally by the node for the response callback. Set to nullptr to use the default one.
     */
    ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group);

    /**
     * @brief A convenience constructor.
     *
     * This is just for convenience when you want to specify the callback group but don't care about QoS.
     *
     * @param node            ROS node handler.
     * @param address         Name of the service.
     * @param callback_group  Callback group used internally by the node for the response callback. Set to nullptr to use the default one.
     */
    ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::CallbackGroup::SharedPtr& callback_group);

    /**
     * @brief Synchronous (blocking) call of the service.
     *
     * This method will block until the service call returns.
     *
     * @param request The service request to be sent with the call
     *
     * @return Optional shared pointer to the result or std::nullopt if the call failed
     *
     * @warning Take care when using this method! Make sure that the thread and callback group that you're calling it from
     * is not the same as the thread/callback group of the service client or the service server being called! This can typically
     * happen if you call this from another callback or if you're using the SingleThreadedExecutor, and will lead to a deadlock.
     */
    std::optional<std::shared_ptr<typename ServiceType::Response>> callSync(const std::shared_ptr<typename ServiceType::Request>& request);

    /**
     * @brief Asynchronous (non-blocking) call of the service.
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
