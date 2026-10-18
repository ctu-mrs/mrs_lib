#include "mrs_lib/service_client.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <functional>
#include <future>
#include <latch>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/service.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include "mrs_lib/coro/runners.hpp"
#include "mrs_lib/coro/task.hpp"
#include "mrs_lib/logger.hpp"
#include "mrs_lib/timer.hpp"
#include "mrs_lib/utility/callback.hpp"
#include "mrs_lib/utility/owning_mutex.hpp"
#include "mrs_lib/utility/scope_cleanup.hpp"

#include "mrs_lib_testing/ros_fixtures.hpp"

template class mrs_lib::ServiceClient<std_srvs::srv::SetBool>;

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
            service_client_(
                mrs_lib::ServiceClientOptions{
                    .node_interfaces = *this,
                },
                "example_service")
      {
      }

      // Coroutine to do the work.
      // This can be called eg. by other coroutines or callbacks.
      mrs_lib::Task<> work(bool val)
      {
        logger_.info("Sending service request with val: {}", val);

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = val;

        auto response_opt = co_await service_client_.call(request);
        logger_.info("Received response.");
        if (response_opt.has_value())
        {
          auto&& response = response_opt.value();
          logger_.info("Received response with message: '{}'", response->message);
          last_received_response_message_.store(response->message);
        } else
        {
          logger_.error("Empty response!");
        }
      }

      std::string get_last_received_response_message()
      {
        return last_received_response_message_.load();
      }

    private:
      mrs_lib::Logger logger_;

      mrs_lib::OwningMutex<std::string> last_received_response_message_;

      mrs_lib::ServiceClient<std_srvs::srv::SetBool> service_client_;
    };
    // DOCS: END EXAMPLE

    class ServiceClientExampleTest : public mrs_lib_testing::RosExecutorFixture<>
    {
    };


    TEST_F(ServiceClientExampleTest, Example)
    {
      using namespace std::chrono_literals;

      auto node = std::make_shared<ExampleNode>();
      get_executor().add_node(node);

      std::atomic<bool> last_received = false;

      std::function callback = [&](std::shared_ptr<const std_srvs::srv::SetBool::Request> request,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void {
        last_received = request->data;
        response->success = true;
        response->message = std::format("Successfully received: '{}'", request->data);
      };

      auto service_server = node->create_service<std_srvs::srv::SetBool>("example_service", callback);

      // Give some time to discover the service server.
      std::this_thread::sleep_for(50ms);

      std::latch stop_test(1);

      mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<> {
        co_await node->work(true);
        stop_test.count_down();
      });

      stop_test.wait();

      EXPECT_EQ(node->get_last_received_response_message(), "Successfully received: 'true'");
      EXPECT_EQ(last_received, true);

      get_executor().remove_node(node);
    }

  } // namespace example


  using namespace std::chrono_literals;

  class ServiceClientTest : public mrs_lib_testing::RosExecutorFixture<>
  {
  public:
    void callbackService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void callbackRepeatedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void callbackFailedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void SetUp() override
    {
      node_ = std::make_shared<rclcpp::Node>("test_service_client", rclcpp::NodeOptions().use_intra_process_comms(false));

      get_executor().add_node(node_);
    }

    void TearDown() override
    {
      get_executor().remove_node(node_);
    }

    rclcpp::Node::SharedPtr node_;
  };

  void ServiceClientTest::callbackService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {

    response->success = true;

    node_->get_clock()->sleep_for(1s);

    if (request->data)
    {
      response->message = "set";
    } else
    {
      response->message = "unset";
    }
  }

  void ServiceClientTest::callbackFailedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {

    response->success = true;

    if (request->data)
    {
      response->message = "set";
    } else
    {
      response->message = "unset";
    }
  }

  enum class ClientCallbackType
  {
    async,
    coro,
  };

  class ClientTestNode : public rclcpp::Node
  {
  public:
    ClientTestNode(ClientCallbackType type)
        : rclcpp::Node("async_call_node", rclcpp::NodeOptions().use_intra_process_comms(false)),
          logger_(*this),
          reentrant_callback_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
          client_(mrs_lib::ServiceClientOptions{.node_interfaces = *this}, "service1"),
          timer_(create_timer(type))
    {
    }

    bool is_done()
    {
      return done_;
    }

  private:
    mrs_lib::Timer create_timer(ClientCallbackType type)
    {
      switch (type)
      {
      case ClientCallbackType::async:
        return mrs_lib::Timer(
            mrs_lib::RosTimerOptions{
                .node_interfaces = *this,
            },
            10ms, std::bind_front(&ClientTestNode::timer_async_callback, this));
      case ClientCallbackType::coro:
        return mrs_lib::Timer(
            mrs_lib::RosTimerOptions{
                .node_interfaces = *this,
                .callback_group = reentrant_callback_group_,
            },
            10ms, mrs_lib::CoroCallback(mrs_lib::coro_callback_tags::CancelNewDefault{}, &ClientTestNode::timer_coro_callback, this));
      }
      throw std::logic_error("unhandled value of ClientCallbackType");
    }

    void timer_async_callback()
    {
      if (done_)
      {
        logger_.info("Already done. Waiting to stop...");
        return;
      }

      if (!opt_future_.has_value())
      {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        logger_.info("Calling service.");
        auto res = client_.call_async(request);

        ASSERT_TRUE(res.has_value());
        opt_future_ = std::move(res).value();
      }

      auto&& future = opt_future_.value();
      ASSERT_TRUE(future.valid());
      if (opt_future_->wait_for(0ns) == std::future_status::timeout)
      {
        logger_.info("Waiting for the service response...");
        return;
      }

      auto response = future.get();

      EXPECT_TRUE(response->success);
      EXPECT_EQ(response->message, "set");

      logger_.info("Finished");
      done_ = true;
    }

    mrs_lib::Task<void> timer_coro_callback()
    {
      if (done_)
      {
        logger_.info("Already done. Waiting to stop...");
        co_return;
      }

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;

      logger_.info("Calling service.");
      auto res_opt = co_await client_.call(request);
      logger_.info("Service returned.");

      EXPECT_TRUE(res_opt.has_value());
      if (!res_opt.has_value())
      {
        co_return;
      }

      auto response = res_opt.value();

      EXPECT_TRUE(response->success);
      EXPECT_EQ(response->message, "set");

      logger_.info("Finished");
      done_ = true;
    }

    mrs_lib::Logger logger_;

    std::shared_ptr<rclcpp::CallbackGroup> reentrant_callback_group_;

    std::atomic<bool> done_ = false;
    std::optional<std::future<std::shared_ptr<std_srvs::srv::SetBool::Response>>> opt_future_{};

    mrs_lib::ServiceClient<std_srvs::srv::SetBool> client_;

    mrs_lib::Timer timer_;
  };

  TEST_F(ServiceClientTest, CallAsync)
  {
    auto clock = node_->get_clock();

    // | ----------------- create a service server ---------------- |


    using ServerCallbackType = rclcpp::Service<std_srvs::srv::SetBool>::CallbackType;

    const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
        "/service1", ServerCallbackType(std::bind_front(&ServiceClientTest::callbackService, this)), rclcpp::ServicesQoS());

    // | ----------------- create a service client ---------------- |

    auto client_node = std::make_shared<ClientTestNode>(ClientCallbackType::async);

    get_executor().add_node(client_node);

    while (!client_node->is_done())
    {
      RCLCPP_INFO(node_->get_logger(), "Test running...");
      std::this_thread::sleep_for(100ms);
    }

    get_executor().remove_node(client_node);
  }

  TEST_F(ServiceClientTest, CoroCall)
  {
    auto clock = node_->get_clock();

    // | ----------------- create a service server ---------------- |


    using ServerCallbackType = rclcpp::Service<std_srvs::srv::SetBool>::CallbackType;

    const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
        "/service1", ServerCallbackType(std::bind_front(&ServiceClientTest::callbackService, this)), rclcpp::ServicesQoS());

    // | ----------------- create a service client ---------------- |

    auto client_node = std::make_shared<ClientTestNode>(ClientCallbackType::coro);

    get_executor().add_node(client_node);

    while (!client_node->is_done())
    {
      RCLCPP_INFO(node_->get_logger(), "Test running...");
      std::this_thread::sleep_for(100ms);
    }

    get_executor().remove_node(client_node);
  }

  TEST_F(ServiceClientTest, CoroCallStopTokenCancellation)
  {
    using RequestType = std_srvs::srv::SetBool::Request;
    using ResponseType = std_srvs::srv::SetBool::Response;

    auto clock = node_->get_clock();

    // | ----------------- create a service server ---------------- |

    const auto callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
        "/service1", [this](std::shared_ptr<RequestType> req, std::shared_ptr<ResponseType> res) { callbackService(std::move(req), std::move(res)); },
        rclcpp::ServicesQoS(), callback_group);

    // | ----------------- create a service client ---------------- |

    auto client1 = mrs_lib::ServiceClient<std_srvs::srv::SetBool>(
        mrs_lib::ServiceClientOptions{
            .node_interfaces = *node_,
        },
        "service1");

    ASSERT_TRUE(client1.wait_for_service(std::chrono::seconds(-1)));

    RCLCPP_INFO(node_->get_logger(), "initialized");

    std::atomic<bool> started = false;
    std::atomic<bool> completed = false;
    std::atomic<bool> destroyed = false;

    std::latch stop_test(2);

    const auto test_fun = [&]() -> mrs_lib::Task<> {
      started = true;
      mrs_lib::ScopeCleanup cleanup_all([&] {
        RCLCPP_INFO(node_->get_logger(), "Coroutine destroyed");
        destroyed = true;
        stop_test.count_down();
      });

      auto request = std::make_shared<RequestType>();

      {
        request->data = true;

        RCLCPP_INFO(node_->get_logger(), "Calling service.");

        auto opt_response = co_await client1.call(request);

        ADD_FAILURE() << "This callback should be cancelled by now.";

        if (!opt_response.has_value())
        {
          co_return;
        }

        auto response = opt_response.value();

        EXPECT_TRUE(response);
        EXPECT_TRUE(response->success);
        EXPECT_EQ(response->message, "set");
      }

      RCLCPP_INFO(node_->get_logger(), "finished");

      completed = true;
    };

    auto timer = mrs_lib::Timer(
        mrs_lib::RosTimerOptions{
            .node_interfaces = *node_,
            .oneshot = true,
            .callback_group = callback_group,
        },
        0s, [test_fun, &client1, &stop_test]() -> void {
          std::stop_source stop_source{};
          mrs_lib::coro::internal::start_task(stop_source.get_token(), test_fun);
          stop_source.request_stop();
          size_t pruned_requests = client1.prune_pending_requests();
          EXPECT_EQ(pruned_requests, 0) << "Cancelled request should remove itself.";
          stop_test.count_down();
        });

    stop_test.wait();

    ASSERT_TRUE(started);
    ASSERT_TRUE(destroyed);
    ASSERT_FALSE(completed);
  }

  TEST_F(ServiceClientTest, CoroCallPruneCancellation)
  {
    using RequestType = std_srvs::srv::SetBool::Request;
    using ResponseType = std_srvs::srv::SetBool::Response;

    auto clock = node_->get_clock();

    // | ----------------- create a service server ---------------- |

    const auto callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
        "/service1", [this](std::shared_ptr<RequestType> req, std::shared_ptr<ResponseType> res) { callbackService(std::move(req), std::move(res)); },
        rclcpp::ServicesQoS(), callback_group);

    // | ----------------- create a service client ---------------- |

    auto client1 = mrs_lib::ServiceClient<std_srvs::srv::SetBool>(
        mrs_lib::ServiceClientOptions{
            .node_interfaces = *node_,
        },
        "service1");

    RCLCPP_INFO(node_->get_logger(), "initialized");

    std::atomic<bool> started = false;
    std::atomic<bool> completed = false;
    std::atomic<bool> destroyed = false;

    std::latch stop_test(2);

    const auto test_fun = [&]() -> mrs_lib::Task<> {
      started = true;
      mrs_lib::ScopeCleanup cleanup_all([&] {
        RCLCPP_INFO(node_->get_logger(), "Coroutine destroyed");
        destroyed = true;
        stop_test.count_down();
      });

      auto request = std::make_shared<RequestType>();

      {
        request->data = true;

        RCLCPP_INFO(node_->get_logger(), "Calling service.");

        auto opt_response = co_await client1.call(request);

        ADD_FAILURE() << "This callback should be cancelled by now.";

        if (!opt_response.has_value())
        {
          co_return;
        }

        auto response = opt_response.value();

        EXPECT_TRUE(response);
        EXPECT_TRUE(response->success);
        EXPECT_EQ(response->message, "set");
      }

      RCLCPP_INFO(node_->get_logger(), "finished");

      completed = true;
    };

    auto timer = mrs_lib::Timer(
        mrs_lib::RosTimerOptions{
            .node_interfaces = *node_,
            .oneshot = true,
            .callback_group = callback_group,
        },
        0s, [test_fun, &client1, &stop_test]() -> void {
          mrs_lib::coro::internal::start_task(test_fun);
          size_t prunned_requests = client1.prune_pending_requests();
          EXPECT_EQ(prunned_requests, 1) << "The callback should be pruned by this call.";
          stop_test.count_down();
        });

    stop_test.wait();

    ASSERT_TRUE(started);
    ASSERT_TRUE(destroyed);
    ASSERT_FALSE(completed);
  }

  TEST_F(ServiceClientTest, BadName)
  {

    auto clock = node_->get_clock();

    // | ----------------- create a service client ---------------- |

    auto client1 = mrs_lib::ServiceClient<std_srvs::srv::SetBool>(
        mrs_lib::ServiceClientOptions{
            .node_interfaces = *node_,
        },
        "random");

    RCLCPP_INFO(node_->get_logger(), "initialized");


    std::latch stop_test(2);

    const auto test_fun = [&]() {
      {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

        request->data = true;

        auto response = client1.call_async(request);

        EXPECT_FALSE(response.has_value());
      }

      {
        mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<> {
          auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
          request->data = true;

          auto response = co_await client1.call(request);
          EXPECT_FALSE(response.has_value());

          stop_test.count_down();

          co_return;
        });
      }

      RCLCPP_INFO(node_->get_logger(), "finished");

      stop_test.count_down();
    };

    auto timer = mrs_lib::Timer(
        mrs_lib::RosTimerOptions{
            .node_interfaces = *node_,
            .oneshot = true,
        },
        0s, test_fun);

    stop_test.wait();
  }

  TEST_F(ServiceClientTest, GetServiceNames)
  {
    auto client = mrs_lib::ServiceClient<std_srvs::srv::SetBool>(
        mrs_lib::ServiceClientOptions{
            .node_interfaces = *node_,
        },
        "~/my_service");

    RCLCPP_INFO(node_->get_logger(), "initialized");

    EXPECT_EQ(client.get_service_name(), "/test_service_client/my_service");
    EXPECT_EQ(client.get_unresolved_service_name(), "~/my_service");
  }

  TEST_F(ServiceClientTest, IsServiceReady)
  {

    auto client = mrs_lib::ServiceClient<std_srvs::srv::SetBool>(
        mrs_lib::ServiceClientOptions{
            .node_interfaces = *node_,
        },
        "/readiness_service");

    RCLCPP_INFO(node_->get_logger(), "initialized");

    // no server is running — should not be ready
    EXPECT_FALSE(client.is_service_ready());

    // | ------------ start a service server and re-check ---------- |

    using ServerCallbackType = rclcpp::Service<std_srvs::srv::SetBool>::CallbackType;

    const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
        "/readiness_service", ServerCallbackType(std::bind_front(&ServiceClientTest::callbackFailedService, this)), rclcpp::ServicesQoS(), svr_grp);

    // give the executor a moment to register the server
    std::this_thread::sleep_for(100ms);

    EXPECT_TRUE(client.is_service_ready());

    RCLCPP_INFO(node_->get_logger(), "finished");
  }

  TEST_F(ServiceClientTest, WaitForService)
  {
    // | ----------------- create a service client ---------------- |

    auto client = mrs_lib::ServiceClient<std_srvs::srv::SetBool>(
        mrs_lib::ServiceClientOptions{
            .node_interfaces = *node_,
        },
        "/wait_service");

    RCLCPP_INFO(node_->get_logger(), "initialized");

    std::latch stop_test(1);

    const auto test_fun = [&]() {
      // no server running — should time out
      EXPECT_FALSE(client.wait_for_service(200ms));

      // | ------------ start a server and wait again ---------------- |

      using ServerCallbackType = rclcpp::Service<std_srvs::srv::SetBool>::CallbackType;

      const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
          "/wait_service", ServerCallbackType(std::bind_front(&ServiceClientTest::callbackFailedService, this)), rclcpp::ServicesQoS(), svr_grp);

      // should become available well within 2 seconds
      EXPECT_TRUE(client.wait_for_service(2s));

      RCLCPP_INFO(node_->get_logger(), "finished");

      stop_test.count_down();
    };

    auto timer = mrs_lib::Timer(
        mrs_lib::RosTimerOptions{
            .node_interfaces = *node_,
            .oneshot = true,
        },
        0s, test_fun);

    stop_test.wait();
  }

} // namespace
