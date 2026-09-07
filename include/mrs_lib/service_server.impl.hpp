#ifndef MRS_LIB_SERVICE_SERVER_IMPL_HPP_
#define MRS_LIB_SERVICE_SERVER_IMPL_HPP_

#include "mrs_lib/service_server.hpp"

#include <cassert>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/create_service.hpp>
#include <rclcpp/service.hpp>

#include "mrs_lib/coro/runners.hpp"
#include "mrs_lib/coro/task.hpp"
#include "mrs_lib/internal/coroutine_callback_helpers.hpp"
#include "mrs_lib/logger.hpp"
#include "mrs_lib/utility/pimpl.hpp"

namespace mrs_lib
{

  /** @private */
  template <class ServiceType>
  class ServiceServer<ServiceType>::Impl
  {
    using RosServiceServerPimpl = Pimpl<rclcpp::Service<ServiceType>, std::shared_ptr<rclcpp::Service<ServiceType>>>;

  public:
    Impl(const ServiceServerOptions& options, std::string_view service_name, ServiceCallback callback)
        : node_interfaces_(options.node_interfaces),
          logger_(node_interfaces_),
          unresolved_service_name_(service_name),
          service_server_(rclcpp::create_service<ServiceType>(node_interfaces_.get_node_base_interface(), node_interfaces_.get_node_services_interface(),
                                                              std::string(service_name), callback, options.qos, options.callback_group)),
          resolved_service_name_(service_server_->get_service_name())
    {
      logger_.info("Created service server '{}' -> '{}'", get_unresolved_service_name(), get_service_name());
    }

    Impl(const ServiceServerOptions& options, std::string_view service_name, ServiceCoroCallback callback)
        : node_interfaces_(options.node_interfaces),
          logger_(node_interfaces_),
          unresolved_service_name_(service_name),
          service_server_(rclcpp::create_service<ServiceType>(node_interfaces_.get_node_base_interface(), node_interfaces_.get_node_services_interface(),
                                                              std::string(service_name), create_coro_callback(callback), options.qos, std::invoke([&]() {
                                                                auto callback_group = options.callback_group;
                                                                internal::require_callback_group_coro_compatible(callback_group);
                                                                return callback_group;
                                                              }))),
          resolved_service_name_(service_server_->get_service_name())
    {
      logger_.info("Created service server '{}' -> '{}'", get_unresolved_service_name(), get_service_name());
    }

    ~Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    [[nodiscard]] std::string get_service_name() const
    {
      return resolved_service_name_;
    }

    [[nodiscard]] std::string get_unresolved_service_name() const
    {
      return unresolved_service_name_;
    }

  private:
    static std::function<void(std::shared_ptr<rclcpp::Service<ServiceType>> service_server, std::shared_ptr<rmw_request_id_t> header,
                              std::shared_ptr<const Request> request)>
    create_coro_callback(ServiceCoroCallback callback)
    {
      return [callback](std::shared_ptr<rclcpp::Service<ServiceType>> service_server, std::shared_ptr<rmw_request_id_t> header,
                        std::shared_ptr<const Request> request) {
        assert(service_server != nullptr);
        assert(header != nullptr);
        assert(request != nullptr);
        coro::internal::start_task([callback, service_server, header, request]() -> mrs_lib::Task<> {
          auto response = std::make_shared<Response>();
          co_await callback(request, response);
          service_server->send_response(*header, *response);
        });
      };
    }

    ServiceServerNodeInterfaces node_interfaces_;
    mrs_lib::Logger logger_;

    std::string unresolved_service_name_;

    RosServiceServerPimpl service_server_;

    // rclcpp does not currently expose get_service_name as const.
    // Fix in: ros2/rclcpp#3253
    // We cache the value to be usable in const methods.
    std::string resolved_service_name_;
  };

  template <class ServiceType>
  ServiceServer<ServiceType>::ServiceServer(const ServiceServerOptions& options, std::string_view service_name, ServiceCallback callback)
      : impl_(std::in_place_type_t<Impl>{}, options, service_name, callback)
  {
  }

  template <class ServiceType>
  ServiceServer<ServiceType>::ServiceServer(const ServiceServerOptions& options, std::string_view service_name, ServiceCoroCallback callback)
      : impl_(std::in_place_type_t<Impl>{}, options, service_name, callback)
  {
  }

  template <class ServiceType>
  std::string ServiceServer<ServiceType>::get_service_name() const
  {
    return impl_->get_service_name();
  }

  template <class ServiceType>
  std::string ServiceServer<ServiceType>::get_unresolved_service_name() const
  {
    return impl_->get_unresolved_service_name();
  }

} // namespace mrs_lib


#endif // MRS_LIB_SERVICE_SERVER_IMPL_HPP_
