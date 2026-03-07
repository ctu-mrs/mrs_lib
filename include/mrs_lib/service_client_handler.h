/**  \file
     \brief Defines ServiceClientHandler and related convenience classes for upgrading the ROS service client
     \author Tomas Baca - tomas.baca@fel.cvut.cz
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <future>

#include <mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp>
#include <mrs_lib/coro/task.hpp>

namespace mrs_lib
{
  template <class ServiceType>
  class ServiceClientHandler;

  namespace internal
  {
    template <typename ServiceType>
      requires(rosidl_generator_traits::is_service<ServiceType>::value)
    class [[nodiscard("This service call is only performed when `co_await`ed.")]] ServiceAwaitable;
  }

  /* class ServiceClientHandler //{ */

  /**
   * @brief user wrapper of the service client handler implementation
   */
  template <class ServiceType>
  class ServiceClientHandler
  {

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
    ServiceClientHandler(rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos,
                         const rclcpp::CallbackGroup::SharedPtr& callback_group);

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

    /**
     * @brief Awaitable call of the service.
     *
     * This function can be used inside coroutine callbacks (mrs_lib::Task) to
     * suspend the coroutine until the service completes. The advantage of this
     * method compared to callSync is that this can run on single threaded
     * executor - the calling coroutine is suspended and the executor can work
     * on other tasks. Once the service responds, the calling coroutine is
     * resumed and the co_await expression returns
     * `std::optional<std::shared_ptr<Response>>` where `Response` is the
     * service response type.
     *
     * @param request The service request to be sent with the call
     *
     * @return Awaitable type that calls the service and returns the result when awaited.
     */
    Task<std::optional<std::shared_ptr<typename ServiceType::Response>>> callAwaitable(std::shared_ptr<typename ServiceType::Request> request);

    /**
     * @brief Returns the name of the service this client connects to.
     *
     * @return service name, or an empty string if the handler is not initialized
     */
    std::string getServiceName() const;

    /**
     * @brief Waits for the service to be available.
     *
     * @tparam RepT   arithmetic type representing the number of ticks (deduced automatically)
     * @tparam RatioT std::ratio representing the tick period (deduced automatically)
     * @param timeout maximum time to wait for the service to be available
     *
     * @return true if the service became available within the timeout, false otherwise
     */
    template <typename RepT = int64_t, typename RatioT = std::milli>
    bool waitForService(std::chrono::duration<RepT, RatioT> timeout = std::chrono::seconds(1));

    /**
     * @brief Checks if the service is available.
     *
     * @return true if the service is available, false otherwise
     */
    bool isServiceReady() const;

  private:
    class Impl;
    std::shared_ptr<Impl> impl_;
  };

  //}

} // namespace mrs_lib

#include <mrs_lib/impl/service_client_handler.hpp>
