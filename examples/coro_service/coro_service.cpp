#include <std_srvs/srv/set_bool.hpp>
#include <mrs_lib/coro/task.hpp>
#include <mrs_lib/node.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/timer_handler.h>


namespace mrs_lib_examples
{
  using namespace std::chrono_literals;
  using ServiceType = std_srvs::srv::SetBool;

  class CoroServiceExample : public mrs_lib::Node
  {
  public:
    // BEGIN CTOR
    CoroServiceExample(const rclcpp::NodeOptions& opts)
        : Node("coro_service_example", opts),
          logger(this_node().get_logger()),
          reentrant_callback_group_(this_node().create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
          service_client_(this_node_ptr(), "my_service", nullptr),
          service_timer_(std::make_unique<mrs_lib::ROSTimer>(std::invoke([this] {
                                                               mrs_lib::TimerHandlerOptions opts{this_node_ptr()};
                                                               opts.callback_group = reentrant_callback_group_;
                                                               return opts;
                                                             }),
                                                             rclcpp::Rate(5s), &CoroServiceExample::service_timer_callback, this)),
          chatter_timer_(
              std::make_unique<mrs_lib::ROSTimer>(this_node_ptr(), rclcpp::Rate(1s), std::bind_front(&CoroServiceExample::chatter_timer_callback, this)))
    {
    }
    // END CTOR

  private:
    // BEGIN CORO
    mrs_lib::Task<std::optional<std::shared_ptr<ServiceType::Response>>> call_service(bool data)
    {
      auto request = std::make_shared<ServiceType::Request>();
      request->data = data;
      co_return co_await service_client_.callAwaitable(request);
    }
    // END CORO

    // BEGIN CORO CALLBACK
    mrs_lib::Task<> service_timer_callback()
    {
      RCLCPP_INFO_STREAM(logger, "Calling service...");
      std::optional<std::shared_ptr<ServiceType::Response>> response_opt = co_await call_service(true);
      if (response_opt.has_value())
      {
        auto response = response_opt.value();
        std::string response_str = std::format("success: '{}'\n  message: '{}'", response->success, response->message);
        RCLCPP_INFO_STREAM(logger, "Service response:\n" << response_str);
      } else
      {
        RCLCPP_WARN_STREAM(logger, "Failed to call service!!!");
      }
    }
    // END CORO CALLBACK


    void chatter_timer_callback()
    {
      RCLCPP_INFO_STREAM(logger, "Chattering ... (" << message_number_ << ")");
      message_number_ += 1;
    }

    rclcpp::Logger logger;

    size_t message_number_ = 0;

    std::shared_ptr<rclcpp::CallbackGroup> reentrant_callback_group_;
    mrs_lib::ServiceClientHandler<ServiceType> service_client_;
    std::unique_ptr<mrs_lib::MRSTimer> service_timer_;
    std::unique_ptr<mrs_lib::MRSTimer> chatter_timer_;
  };


  class CoroServiceExampleServer : public mrs_lib::Node
  {
  public:
    CoroServiceExampleServer(const rclcpp::NodeOptions& opts)
        : Node("coro_service_example_server", opts),
          logger(this_node().get_logger()),
          service_server_(this_node_ptr(), "my_service", std::bind_front(&CoroServiceExampleServer::service_callback, this))
    {
    }

  private:
    void service_callback(std::shared_ptr<ServiceType::Request> req, std::shared_ptr<ServiceType::Response> res)
    {
      RCLCPP_INFO_STREAM(logger, "Received service call. Starting work...");

      // Simulated work...
      // Warning: This blocks the executor.
      // If this runs on a single threaded executor, no other callbacks
      // on ANY node in this executor will be called until this one finishes.
      // In the case of this example, it is ok, but you should be aware of this problem.
      std::this_thread::sleep_for(1s);
      res->success = true;
      res->message = std::format("Response to service with value: '{}'", req->data);

      RCLCPP_INFO_STREAM(logger, "Work done. Sending response...");
    }

    rclcpp::Logger logger;

    mrs_lib::ServiceServerHandler<ServiceType> service_server_;
  };

} // namespace mrs_lib_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_lib_examples::CoroServiceExample)
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_lib_examples::CoroServiceExampleServer)
