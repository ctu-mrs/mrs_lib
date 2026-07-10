/**  \file
     \brief Implements ServiceClientHandler and related convenience classes for upgrading the ROS service client
     \author Tomas Baca - tomas.baca@fel.cvut.cz
     \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#pragma once

#include <mrs_lib/coro/cancellation.hpp>
#include <mrs_lib/coro/event.hpp>
#include <mrs_lib/service_client_handler.h>

namespace mrs_lib
{

  // --------------------------------------------------------------
  // |                    ServiceClientHandler                    |
  // --------------------------------------------------------------

  /* ServiceClientHandler() constructors //{ */

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(const rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos)
      : impl_(std::make_shared<Impl>(node, address, qos, node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)))
  {
  }

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler() : impl_(nullptr)
  {
  }

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(const rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos,
                                                          const rclcpp::CallbackGroup::SharedPtr& callback_group)
      : impl_(std::make_shared<Impl>(node, address, qos, callback_group))
  {
  }

  template <class ServiceType>
  ServiceClientHandler<ServiceType>::ServiceClientHandler(const rclcpp::Node::SharedPtr& node, const std::string& address,
                                                          const rclcpp::CallbackGroup::SharedPtr& callback_group)
      : ServiceClientHandler(node, address, rclcpp::ServicesQoS(), callback_group)
  {
  }

  //}

  /* callSync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  std::optional<std::shared_ptr<typename ServiceType::Response>>
  ServiceClientHandler<ServiceType>::callSync(const std::shared_ptr<typename ServiceType::Request>& request)
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use callSync()!");
      return std::nullopt;
    }
    return impl_->callSync(request);
  }

  //}

  /* callAsync(const ServiceType::Request& request, ServiceType::Response& response) //{ */

  template <class ServiceType>
  std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>>
  ServiceClientHandler<ServiceType>::callAsync(const std::shared_ptr<typename ServiceType::Request>& request)
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use callAsync()!");
      return std::nullopt;
    }
    return impl_->callAsync(request);
  }

  //}

  template <class ServiceType>
  Task<std::optional<std::shared_ptr<typename ServiceType::Response>>>
  ServiceClientHandler<ServiceType>::callAwaitable(std::shared_ptr<typename ServiceType::Request> request)
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use callAwaitable()!");
      co_return std::nullopt;
    }

    co_return co_await impl_->callAwaitable(request);
  }

  /* getServiceName() //{ */

  template <class ServiceType>
  std::string ServiceClientHandler<ServiceType>::getServiceName() const
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use getServiceName()!");
      return {};
    }
    return impl_->getServiceName();
  }

  //}

  /* waitForService() //{ */

  template <class ServiceType>
  template <typename RepT, typename RatioT>
  bool ServiceClientHandler<ServiceType>::waitForService(std::chrono::duration<RepT, RatioT> timeout)
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use waitForService()!");
      return false;
    }
    return impl_->waitForService(timeout);
  }

  //}

  /* isServiceReady() //{ */
  template <class ServiceType>
  bool ServiceClientHandler<ServiceType>::isServiceReady() const
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use isServiceReady()!");
      return false;
    }
    return impl_->isServiceReady();
  }

  //}

  template <class ServiceType>
  size_t ServiceClientHandler<ServiceType>::prunePendingRequests()
  {
    if (!impl_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ServiceClientHandler"), "Not initialized, cannot use prunePendingRequests()!");
      return false;
    }
    return impl_->prunePendingRequests();
  }


  // --------------------------------------------------------------
  // |                 ServiceClientHandler::Impl                 |
  // --------------------------------------------------------------

  /* class ServiceClientHandler::impl //{ */

  /**
   * @brief implementation of the service client handler
   */
  template <class ServiceType>
  class ServiceClientHandler<ServiceType>::Impl
  {

  public:
    /**
     * @brief constructor
     *
     * @param node ROS node handler
     * @param address service address
     * @param qos QOS
     * @param callback_group callback group
     */
    Impl(const rclcpp::Node::SharedPtr& node, const std::string& address, const rclcpp::QoS& qos, const rclcpp::CallbackGroup::SharedPtr& callback_group)
        : callback_group_(callback_group), service_client_(node->create_client<ServiceType>(address, qos, callback_group))
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Created client '" << address << "' -> '" << service_client_->get_service_name() << "'");
    }

    /**
     * @brief "classic" synchronous service call
     *
     * @param request request
     *
     * @return optional shared pointer to the response
     */
    std::optional<std::shared_ptr<typename ServiceType::Response>> callSync(const std::shared_ptr<typename ServiceType::Request>& request)
    {
      /* always check if the service is ready before calling */
      if (!service_client_->service_is_ready())
        return std::nullopt;

      /* the future done callback is being run in a separate thread under the default callback group of the node */
      const auto future_msg = service_client_->async_send_request(request).future.share();

      /* it is a good practice to check if the future object is not already invalid after the call */
      /* if valid() is false, the future has UNDEFINED behavior */
      if (!future_msg.valid())
        return std::nullopt;

      // wait for the future to become available and then return
      return future_msg.get();
    }

    /**
     * @brief asynchronous service call
     *
     * @param request request
     *
     * @return optional shared future to the result
     */
    std::optional<std::shared_future<std::shared_ptr<typename ServiceType::Response>>> callAsync(const std::shared_ptr<typename ServiceType::Request>& request)
    {
      /* always check if the service is ready before calling */
      if (!service_client_->service_is_ready())
        return std::nullopt;

      const auto future = service_client_->async_send_request(request).future.share();

      /* it is a good practice to check if the future object is not already invalid after the call */
      /* if valid() is false, the future has UNDEFINED behavior */
      if (!future.valid())
        return std::nullopt;

      return future;
    }

    Task<std::optional<std::shared_ptr<typename ServiceType::Response>>> callAwaitable(const std::shared_ptr<typename ServiceType::Request>& request)
    {
      using Response = ServiceType::Response;
      using Client = rclcpp::Client<ServiceType>;
      using SharedFutureAndRequestId = Client::SharedFutureAndRequestId;
      using StopTokenBehavior = coro::internal::LowLevelEventAwaitable::StopTokenBehavior;

      struct StateData
      {
        std::mutex mutex{};
        bool cancelled = false;

        std::optional<SharedFutureAndRequestId> future_and_id{};
      };

      if (!service_client_->service_is_ready())
      {
        co_return std::nullopt;
      }

      auto [event, awaitable] = coro::make_event();

      std::shared_ptr<StateData> state_data = std::make_shared<StateData>();

      auto register_waker = [&event, client = service_client_, request, state_data]() {
        std::lock_guard lock(state_data->mutex);
        if (state_data->cancelled)
        {
          event.try_cancel();
          return;
        }
        auto shared_event = std::make_shared<coro::Event>(std::move(event));
        state_data->future_and_id =
            client->async_send_request(request, [shared_event](std::shared_future<std::shared_ptr<Response>>) { shared_event->try_trigger(); });
      };

      auto low_level_awaitable = coro::internal::get_low_level_event_awaitable(std::move(awaitable));

      {
        // If cancellation is requested via stop token, this callback sets
        // cancelled flag on the state and removes a the pending request.
        // The flag is set to stop sending the request in case the stop token
        // is triggered before the service is called.
        std::stop_callback remove_request_if_canceled(co_await coro::get_task_stop_token(), [state_data, client_weak = std::weak_ptr(service_client_)]() {
          std::lock_guard lock(state_data->mutex);
          state_data->cancelled = true;
          std::shared_ptr<Client> client = client_weak.lock();
          if (client == nullptr || !state_data->future_and_id.has_value())
          {
            return;
          }
          client->remove_pending_request(state_data->future_and_id.value());
        });

        co_await std::move(low_level_awaitable).get_awaitable(StopTokenBehavior::ignore, register_waker);
      }

      {
        std::lock_guard lock(state_data->mutex);
        if (!state_data->future_and_id.has_value())
        {
          co_return std::nullopt;
        }

        auto future = state_data->future_and_id.value().future;
        if (future.wait_for(std::chrono::nanoseconds(0)) == std::future_status::timeout)
        {
          co_return std::nullopt;
        }

        co_return std::shared_ptr<Response>(future.get());
      }
    }

    /**
     * @brief Returns the name of the service this client connects to
     *
     * @return service name
     */
    std::string getServiceName() const
    {
      return service_client_->get_service_name();
    }

    /**
     * @brief Waits for the service to be available
     *
     * @param timeout maximum time to wait for the service
     *
     * @return true if the service is available, false otherwise
     */
    template <typename RepT = int64_t, typename RatioT = std::milli>
    bool waitForService(std::chrono::duration<RepT, RatioT> timeout)
    {
      return service_client_->wait_for_service(timeout);
    }

    /**
     * @brief Checks if the service is available
     *
     * @return true if the service is available, false otherwise
     */
    bool isServiceReady() const
    {
      return service_client_->service_is_ready();
    }

    /**
     * @brief Clean all pending requests.
     *
     * @return number of pending requests that were removed
     */
    size_t prunePendingRequests() const
    {
      return service_client_->prune_pending_requests();
    }

  private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    typename rclcpp::Client<ServiceType>::SharedPtr service_client_;
  };

  //}

} // namespace mrs_lib
