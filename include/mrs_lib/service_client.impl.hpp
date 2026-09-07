#ifndef MRS_LIB_SERVICE_CLIENT_IMPL_HPP_
#define MRS_LIB_SERVICE_CLIENT_IMPL_HPP_

#include "mrs_lib/service_client.hpp"

#include <expected>
#include <future>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include <rclcpp/client.hpp>
#include <rclcpp/create_client.hpp>

#include "mrs_lib/coro/cancellation.hpp"
#include "mrs_lib/coro/event.hpp"
#include "mrs_lib/logger.hpp"
#include "mrs_lib/utility/pimpl.hpp"

namespace mrs_lib
{

  /** @private */
  template <class ServiceType>
  class ServiceClient<ServiceType>::Impl
  {
  public:
    Impl(const ServiceClientOptions& options, std::string_view service_name)
        : node_interfaces_(options.node_interfaces),
          logger_(node_interfaces_),
          unresolved_service_name_(service_name),
          service_client_(rclcpp::create_client<ServiceType>(node_interfaces_.get_node_base_interface(), node_interfaces_.get_node_graph_interface(),
                                                             node_interfaces_.get_node_services_interface(), std::string(service_name), options.qos,
                                                             options.callback_group))
    {
      logger_.info("Created client of service '{}' -> '{}'", get_unresolved_service_name(), get_service_name());
    }

    ~Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    Task<std::expected<std::shared_ptr<Response>, std::string>> call(const std::shared_ptr<Request>& request)
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
        co_return std::unexpected("Service not ready.");
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
          co_return std::unexpected("No future stored after resume.");
        }

        auto future = state_data->future_and_id.value().future;
        if (future.wait_for(std::chrono::nanoseconds(0)) == std::future_status::timeout)
        {
          co_return std::unexpected("Future not ready after resume.");
        }

        co_return std::shared_ptr<Response>(future.get());
      }
    }

    std::expected<std::future<std::shared_ptr<Response>>, std::string> call_async(const std::shared_ptr<Request>& request)
    {
      if (!service_client_->service_is_ready())
      {
        return std::unexpected("Service not ready.");
      }

      auto future = service_client_->async_send_request(request).future;

      if (!future.valid())
      {
        return std::unexpected("Future not valid.");
      }

      return future;
    }

    [[nodiscard]] std::string get_service_name() const
    {
      return service_client_->get_service_name();
    }

    [[nodiscard]] std::string get_unresolved_service_name() const
    {
      return unresolved_service_name_;
    }

    template <typename RepT = int64_t, typename RatioT = std::milli>
    [[nodiscard]] bool wait_for_service(std::chrono::duration<RepT, RatioT> timeout)
    {
      return service_client_->wait_for_service(timeout);
    }

    [[nodiscard]] bool is_service_ready() const
    {
      return service_client_->service_is_ready();
    }

    size_t prune_pending_requests()
    {
      return service_client_->prune_pending_requests();
    }

  private:
    ServiceClientNodeInterfaces node_interfaces_;
    mrs_lib::Logger logger_;

    std::string unresolved_service_name_;

    std::shared_ptr<rclcpp::Client<ServiceType>> service_client_;
  };


  template <class ServiceType>
  ServiceClient<ServiceType>::ServiceClient(const ServiceClientOptions& options, std::string_view service_name)
      : impl_(std::in_place_type_t<Impl>{}, options, service_name)
  {
  }

  template <class ServiceType>
  auto ServiceClient<ServiceType>::call(const std::shared_ptr<Request>& request) -> Task<std::expected<std::shared_ptr<Response>, std::string>>
  {
    co_return co_await impl_->call(request);
  }

  template <class ServiceType>
  auto ServiceClient<ServiceType>::call_async(const std::shared_ptr<Request>& request) -> std::expected<std::future<std::shared_ptr<Response>>, std::string>
  {
    return impl_->call_async(request);
  }

  template <class ServiceType>
  std::string ServiceClient<ServiceType>::get_service_name() const
  {
    return impl_->get_service_name();
  }

  template <class ServiceType>
  std::string ServiceClient<ServiceType>::get_unresolved_service_name() const
  {
    return impl_->get_unresolved_service_name();
  }

  template <class ServiceType>
  template <typename RepT, typename RatioT>
  bool ServiceClient<ServiceType>::wait_for_service(std::chrono::duration<RepT, RatioT> timeout)
  {
    return impl_->wait_for_service(timeout);
  }

  template <class ServiceType>
  bool ServiceClient<ServiceType>::is_service_ready() const
  {
    return impl_->is_service_ready();
  }

  template <class ServiceType>
  size_t ServiceClient<ServiceType>::prune_pending_requests()
  {
    return impl_->prune_pending_requests();
  }

} // namespace mrs_lib


#endif // MRS_LIB_SERVICE_CLIENT_IMPL_HPP_
