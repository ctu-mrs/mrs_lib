#ifndef MRS_LIB_SERVICE_CLIENT_HPP_
#define MRS_LIB_SERVICE_CLIENT_HPP_

#include <expected>
#include <future>
#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/qos.hpp>

#include "mrs_lib/coro/task.hpp"
#include "mrs_lib/utility/pimpl.hpp"

namespace mrs_lib
{

  /**
   * @brief ROS node interfaces required by the ServiceClient.
   */
  using ServiceClientNodeInterfaces =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeServicesInterface, rclcpp::node_interfaces::NodeGraphInterface,
                                              rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeLoggingInterface,
                                              rclcpp::node_interfaces::NodeClockInterface>;

  /**
   * @brief Options for configuring the ServiceClient.
   */
  struct ServiceClientOptions
  {
    /** @brief ROS node interfaces used by the service client. */
    ServiceClientNodeInterfaces node_interfaces;

    /** @brief QOS settings for the service client. */
    rclcpp::QoS qos = rclcpp::ServicesQoS();
    /** @brief ROS callback group used by the service client callbacks. */
    std::shared_ptr<rclcpp::CallbackGroup> callback_group = nullptr;
  };

  /**
   * @brief rclcpp::Client wrapper with coroutines support.
   *
   * @tparam ServiceType Type of the service this client will call.
   *
   * When using services, it is often useful to wait for the response right after sending it.
   * ServiceClient makes this pattern simpler by allowing the use of coroutines.
   * It provides @ref ServiceClient::call, a coroutine interface for calling the service.
   */
  template <class ServiceType>
  class ServiceClient
  {
  private:
    class Impl;

    using Request = ServiceType::Request;
    using Response = ServiceType::Response;

  public:
    /**
     * @brief Construct the client with the specified options.
     *
     * @param options Options to configure the service client.
     * @param service_name The name of the service to call using this client.
     */
    ServiceClient(const ServiceClientOptions& options, std::string_view service_name);

    /**
     * @brief Coroutine call of the service.
     *
     * @param request Request to send to the service server.
     *
     * @return Response of the server if the call succeeded, error message otherwise.
     */
    Task<std::expected<std::shared_ptr<Response>, std::string>> call(const std::shared_ptr<Request>& request);

    /**
     * @brief Async call of the service.
     *
     * @param request Request to send to the service server.
     *
     * @return Future that will contain the server response if successful, error message otherwise.
     *
     * When the future is returned, it is up to the user to check, if the service server responded.
     * This can be done for example by checking the readiness of the future in a timer callback.
     *
     * @warning If the future is not ready, calling `get` on the future will
     * block the current thread until the response is sent.
     * This may cause deadlocks!
     * (Eg. when service client is on the same mutually exclusive callback group
     * as the currently running function)
     */
    std::expected<std::future<std::shared_ptr<Response>>, std::string> call_async(const std::shared_ptr<Request>& request);

    /**
     * @brief Returns the resolved (full) name of the service handled by this service client.
     *
     * @return The name of the handled service.
     */
    [[nodiscard]] std::string get_service_name() const;

    /**
     * @brief Returns the unresolved name of the service handled by this service client.
     *
     * @return The name of the handled service.
     */
    [[nodiscard]] std::string get_unresolved_service_name() const;

    /**
     * @brief Wait for the service to become available.
     *
     * @tparam RepT Arithmetic type representing the number of ticks.
     * @tparam RatioT std::ratio representing the tick period.
     * @param timeout Maximum time to wait for the service to be available.
     *
     * @return `true` if the service became available within the timeout, `false` otherwise.
     */
    template <typename RepT = int64_t, typename RatioT = std::milli>
    [[nodiscard]] bool wait_for_service(std::chrono::duration<RepT, RatioT> timeout);

    /**
     * @brief Check if the service is available.
     *
     * @return `true` if the service is available, `false` otherwise.
     */
    [[nodiscard]] bool is_service_ready() const;

    /**
     * @brief Remove all pending requests.
     *
     * @return Number of pending requests that were removed.
     *
     * @warning
     * Calling this will not wake up threads that are waiting on `future::get`.
     * These threads will be deadlocked, unless woken by some other means.
     * Coroutines waiting on @ref ServiceClient::call will be cancelled.
     */
    size_t prune_pending_requests();

  private:
    Pimpl<Impl> impl_;
  };

} // namespace mrs_lib

#ifndef MRS_LIB_SERVICE_CLIENT_IMPL_HPP_
#include "mrs_lib/service_client.impl.hpp"
#endif


#endif // MRS_LIB_SERVICE_CLIENT_HPP_
