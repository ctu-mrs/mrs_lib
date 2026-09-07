#include "mrs_lib/service_server.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <format>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <utility>

#include <rclcpp/node.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include "mrs_lib/coro/event.hpp"
#include "mrs_lib/timer.hpp"
#include "mrs_lib/utility/owning_mutex.hpp"
#include "mrs_lib/utility/scope_cleanup.hpp"

#include "mrs_lib_testing/ros_fixtures.hpp"

namespace
{
  namespace example
  {

    // DOCS: BEGIN EXAMPLE
    class ExampleNode : public rclcpp::Node
    {
    public:
      ExampleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
          : Node("example_node", options),
            logger_(*this),
            service_server_(mrs_lib::ServiceServerOptions{.node_interfaces = *this}, "example_service",
                            std::bind_front(&ExampleNode::service_server_callback, this))
      {
      }

      [[nodiscard]] bool get_last_received() const
      {
        return last_received_;
      }

    private:
      void service_server_callback(std::shared_ptr<const std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        logger_.info("Service called with: {}", request->data);
        last_received_ = request->data;
        response->success = true;
        response->message = std::format("Successfully received: '{}'", request->data);
      }

      mrs_lib::Logger logger_;

      std::atomic<bool> last_received_ = false;

      mrs_lib::ServiceServer<std_srvs::srv::SetBool> service_server_;
    };
    // DOCS: END EXAMPLE

    class ServiceServerExampleTest : public mrs_lib_testing::RosExecutorFixture<>
    {
    };

    TEST_F(ServiceServerExampleTest, Example)
    {
      using namespace std::chrono_literals;

      auto node = std::make_shared<ExampleNode>();
      get_executor().add_node(node);

      auto client = node->create_client<std_srvs::srv::SetBool>("example_service");
      ASSERT_TRUE(client->wait_for_service(1s));

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      auto call_res = client->async_send_request(request);
      auto future = std::move(call_res.future);
      ASSERT_TRUE(future.valid());

      auto wait_res = future.wait_for(1s);
      ASSERT_NE(wait_res, std::future_status::timeout);

      auto response = future.get();

      EXPECT_EQ(response->success, true);
      EXPECT_EQ(response->message, "Successfully received: 'true'");
      EXPECT_EQ(node->get_last_received(), true);

      get_executor().remove_node(node);
    }

  } // namespace example

  using namespace std::chrono_literals;

  class ServiceServerTest : public mrs_lib_testing::RosExecutorFixture<>
  {
  public:
    void SetUp() override
    {
      node_ = std::make_shared<rclcpp::Node>("test_service_server", rclcpp::NodeOptions().use_intra_process_comms(false));

      get_executor().add_node(node_);
    }

    void TearDown() override
    {
      get_executor().remove_node(node_);
    }

    rclcpp::Node::SharedPtr node_;
  };

  enum class ServiceCallbackType
  {
    func,
    coro,
  };

  class ServerTestNode : public rclcpp::Node
  {
  public:
    ServerTestNode(ServiceCallbackType type)
        : rclcpp::Node("async_call_node", rclcpp::NodeOptions().use_intra_process_comms(false)),
          logger_(*this),
          reentrant_callback_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
          server_(create_service_server(type)),
          timer_(
              mrs_lib::RosTimerOptions{
                  .node_interfaces = *this,
              },
              100ms, std::bind_front(&ServerTestNode::timer_callback, this))
    {
    }

  private:
    mrs_lib::ServiceServer<std_srvs::srv::SetBool> create_service_server(ServiceCallbackType type)
    {
      switch (type)
      {
      case ServiceCallbackType::func:
        return mrs_lib::ServiceServer<std_srvs::srv::SetBool>(
            mrs_lib::ServiceServerOptions{
                .node_interfaces = *this,
            },
            "service1", std::bind_front(&ServerTestNode::service_callback, this));
      case ServiceCallbackType::coro:
        return mrs_lib::ServiceServer<std_srvs::srv::SetBool>(
            mrs_lib::ServiceServerOptions{
                .node_interfaces = *this,
                .callback_group = reentrant_callback_group_,
            },
            "service1", mrs_lib::CoroCallback(mrs_lib::coro_callback_tags::CancelNewDefault{}, &ServerTestNode::service_coro_callback, this));
      }
      throw std::logic_error("unhandled value of ServiceCallbackType");
    }

    void service_callback(std::shared_ptr<const std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      response->success = true;

      this->get_clock()->sleep_for(1s);

      if (request->data)
      {
        response->message = "set";
      } else
      {
        response->message = "unset";
      }
    }

    mrs_lib::Task<> service_coro_callback(std::shared_ptr<const std_srvs::srv::SetBool::Request> request,
                                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      auto [event, awaitable] = mrs_lib::coro::make_event();
      {
        auto event_guard = timer_event_.acquire();
        *event_guard = {10, std::move(event)};
      }

      co_await std::move(awaitable).wait();

      response->success = true;
      if (request->data)
      {
        response->message = "set";
      } else
      {
        response->message = "unset";
      }

      co_return;
    }

    void timer_callback()
    {
      {
        auto event_guard = timer_event_.acquire();
        if (!event_guard->has_value())
        {
          return;
        }

        event_guard->value().first -= 1;
        if (event_guard->value().first == 0)
        {
          logger_.info("Triggering event.");
          event_guard->value().second.try_trigger();
          event_guard->reset();
        }
      }
    }

    mrs_lib::Logger logger_;

    std::shared_ptr<rclcpp::CallbackGroup> reentrant_callback_group_;

    std::atomic<bool> done_ = false;

    mrs_lib::OwningMutex<std::optional<std::pair<size_t, mrs_lib::coro::Event>>> timer_event_;

    mrs_lib::ServiceServer<std_srvs::srv::SetBool> server_;

    mrs_lib::Timer timer_;
  };

  TEST_F(ServiceServerTest, Callback)
  {
    auto clock = node_->get_clock();

    // | ----------------- create a service server ---------------- |

    auto service_node = std::make_shared<ServerTestNode>(ServiceCallbackType::func);
    get_executor().add_node(service_node);
    mrs_lib::ScopeCleanup remove_service_node([&, this] { get_executor().remove_node(service_node); });

    // | ----------------- create a service client ---------------- |
    const auto client = node_->create_client<std_srvs::srv::SetBool>("service1", rclcpp::ServicesQoS());

    RCLCPP_INFO(node_->get_logger(), "initialized");


    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto resp_fut = client->async_send_request(request);
    EXPECT_TRUE(resp_fut.valid());

    while (rclcpp::ok())
    {
      RCLCPP_INFO(node_->get_logger(), "waiting for the future response");

      if (resp_fut.wait_for(1s) != std::future_status::timeout)
      {
        break;
      }
    }

    const auto& resp = resp_fut.get();
    EXPECT_TRUE(resp->success);
    EXPECT_EQ(resp->message, "set");
  }

  TEST_F(ServiceServerTest, CoroCallback)
  {
    auto clock = node_->get_clock();

    // | ----------------- create a service server ---------------- |

    auto service_node = std::make_shared<ServerTestNode>(ServiceCallbackType::coro);
    get_executor().add_node(service_node);
    mrs_lib::ScopeCleanup remove_service_node([&, this] { get_executor().remove_node(service_node); });

    // | ----------------- create a service client ---------------- |
    const auto client = node_->create_client<std_srvs::srv::SetBool>("service1", rclcpp::ServicesQoS());

    RCLCPP_INFO(node_->get_logger(), "initialized");


    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto resp_fut = client->async_send_request(request);
    EXPECT_TRUE(resp_fut.valid());

    while (rclcpp::ok())
    {
      RCLCPP_INFO(node_->get_logger(), "waiting for the future response");

      if (resp_fut.wait_for(1s) != std::future_status::timeout)
      {
        break;
      }
    }

    const auto& resp = resp_fut.get();
    EXPECT_TRUE(resp->success);
    EXPECT_EQ(resp->message, "set");
  }

  TEST_F(ServiceServerTest, GetServiceNames)
  {
    auto client = mrs_lib::ServiceServer<std_srvs::srv::SetBool>(
        mrs_lib::ServiceServerOptions{
            .node_interfaces = *node_,
        },
        "~/my_service", [](auto, auto) {});

    EXPECT_EQ(client.get_service_name(), "/test_service_server/my_service");
    EXPECT_EQ(client.get_unresolved_service_name(), "~/my_service");
  }

} // namespace
