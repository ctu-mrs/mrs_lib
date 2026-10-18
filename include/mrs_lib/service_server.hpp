#ifndef MRS_LIB_SERVICE_SERVER_HPP_
#define MRS_LIB_SERVICE_SERVER_HPP_

#include <functional>
#include <memory>
#include <string_view>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/qos.hpp>

#include "mrs_lib/utility/callback.hpp"
#include "mrs_lib/utility/pimpl.hpp"

namespace mrs_lib
{

  /**
   * @brief ROS node interfaces required by the ServiceServer.
   */
  using ServiceServerNodeInterfaces =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeServicesInterface, rclcpp::node_interfaces::NodeBaseInterface,
                                              rclcpp::node_interfaces::NodeLoggingInterface, rclcpp::node_interfaces::NodeClockInterface>;

  /**
   * @brief Options for configuring the ServiceServer.
   */
  struct ServiceServerOptions
  {
    /** @brief ROS node interfaces used by the service server. */
    ServiceServerNodeInterfaces node_interfaces;

    /** @brief QOS settings for the service server. */
    rclcpp::QoS qos = rclcpp::ServicesQoS();
    /** @brief ROS callback group used by the service server callbacks. */
    std::shared_ptr<rclcpp::CallbackGroup> callback_group = nullptr;
  };

  /**
   * @brief rclcpp::Service wrapper with coroutines support.
   *
   * @tparam ServiceType Type of the service this server will provide.
   */
  template <class ServiceType>
  class ServiceServer
  {
    class Impl;

    using Request = ServiceType::Request;
    using Response = ServiceType::Response;

  public:
    /** @brief Type of the accepted callback. */
    using ServiceCallback = std::function<void(std::shared_ptr<const Request> req, std::shared_ptr<Response>)>;

    /** @brief Type of the accepted coroutine callback. */
    using ServiceCoroCallback = CoroCallback<void(std::shared_ptr<const Request> req, std::shared_ptr<Response>)>;

    /**
     * @brief Construct the service server with the specified callback.
     *
     * @brief options Options to configure the service server.
     * @param service_name Name of the service to provide.
     * @param callback Callback to call when a request is received.
     */
    ServiceServer(const ServiceServerOptions& options, std::string_view service_name, ServiceCallback callback);

    /**
     * @brief Construct the service server with the specified coroutine callback.
     *
     * @brief options Options to configure the service server.
     * @param service_name Name of the service to provide.
     * @param callback Coroutine callback to call when a request is received.
     */
    ServiceServer(const ServiceServerOptions& options, std::string_view service_name, ServiceCoroCallback callback);

    /**
     * @brief Returns the resolved (full) name of the service handled by this service server.
     *
     * @return The name of the handled service.
     */
    [[nodiscard]] std::string get_service_name() const;

    /**
     * @brief Returns the unresolved name of the service handled by this service server.
     *
     * @return The name of the handled service.
     */
    [[nodiscard]] std::string get_unresolved_service_name() const;

  private:
    Pimpl<Impl> impl_;
  };

} // namespace mrs_lib


#ifndef MRS_LIB_SERVICE_SERVER_IMPL_HPP_
#include "mrs_lib/service_server.impl.hpp"
#endif


#endif // MRS_LIB_SERVICE_SERVER_HPP_
