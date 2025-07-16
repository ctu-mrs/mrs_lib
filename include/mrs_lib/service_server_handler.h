/**  \file
     \brief Defines the ServiceServerHandler wrapper to ROS2's ServiceServer
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace mrs_lib
{

  /* class ServiceServerHandler //{ */

  /**
   * @brief user wrapper of the service client handler implementation
   */
  template <class ServiceType>
  class ServiceServerHandler
  {

  public:

    using callback_t = typename rclcpp::Service<ServiceType>::CallbackType;

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
    ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk, const rclcpp::QoS& qos = rclcpp::ServicesQoS());

    /*!
     * @brief Default constructor to avoid having to use pointers.
     *
     * It does nothing and the object it constructs will also do nothing.
     * Use some of the other constructors for a construction of an actually usable object.
     */
    ServiceServerHandler();

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
    ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk, const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group);

    /**
     * @brief A convenience constructor.
     *
     * This is just for convenience when you want to specify the callback group but don't care about QoS.
     *
     * @param node            ROS node handler.
     * @param address         Name of the service.
     * @param callback_group  Callback group used internally by the node for the response callback. Set to nullptr to use the default one.
     */
    ServiceServerHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const callback_t& cbk, const rclcpp::CallbackGroup::SharedPtr& callback_group);

  private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    typename rclcpp::Service<ServiceType>::SharedPtr service_server_;
  };

  //}

}  // namespace mrs_lib

#include <mrs_lib/impl/service_server_handler.hpp>
