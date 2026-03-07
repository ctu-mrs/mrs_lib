#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/coro/runners.hpp>
#include <mrs_lib/service_client_handler.h>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

class Test : public ::testing::Test
{

public:
  void callbackService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void callbackRepeatedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void callbackFailedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

protected:
  /* SetUpTestCase() //{ */

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  //}

  /* TearDownTestCase() //{ */

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  //}

  /* initialize() //{ */

  template <typename ExecutorType = rclcpp::executors::MultiThreadedExecutor>
  void initialize(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
  {

    node_ = std::make_shared<rclcpp::Node>("test_service_client_handler", node_options);

    executor_ = std::make_shared<ExecutorType>();
    executor_->add_node(node_);
  }

  //}

  /* despin() //{ */

  void despin()
  {
    executor_->cancel();
  }

  //}

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
};

/* callbackService() //{ */

void Test::callbackService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
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

//}

/* callbackFailedService() //{ */

void Test::callbackFailedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
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

//}

/* TEST_F(Test, test_call) //{ */

TEST_F(Test, test_call)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service server ---------------- |

  // the service server has to be in a different callback group than the timer!
  // otherwise, the callback will never be called, because the default callback
  // group is MutuallyExclusive
  const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
      "/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), svr_grp);

  // | ----------------- create a service client ---------------- |

  // the ServiceClientHandler by default creates its own MutuallyExclusive callback group
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "service1");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() {
    tim->cancel(); // just a one-shot timer

    {

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

      request->data = true;

      auto response = client1.callSync(request);

      EXPECT_TRUE(response.has_value());
      EXPECT_TRUE(response.value()->success);
      EXPECT_EQ(response.value()->message, "set");
    }

    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

      request->data = false;

      auto response = client1.callSync(request);

      EXPECT_TRUE(response.has_value());
      EXPECT_TRUE(response.value()->success);
      EXPECT_EQ(response.value()->message, "unset");
    }

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}

/* TEST_F(Test, asynctest_call) //{ */

TEST_F(Test, asynctest_call)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service server ---------------- |

  // the service server has to be in a different callback group than the timer!
  // otherwise, the callback will never be called, because the default callback
  // group is MutuallyExclusive
  const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
      "/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), svr_grp);

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "service1");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() {
    tim->cancel(); // just a one-shot timer

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    {
      request->data = true;

      auto opt_response = client1.callAsync(request);

      ASSERT_TRUE(opt_response.has_value());

      auto response = opt_response.value();

      while (rclcpp::ok())
      {

        RCLCPP_INFO(node_->get_logger(), "waiting for the future response");

        if (response.wait_for(1s) == std::future_status::ready)
          break;
      }

      EXPECT_TRUE(response.valid());
      EXPECT_TRUE(response.get()->success);
      EXPECT_EQ(response.get()->message, "set");
    }

    RCLCPP_INFO(node_->get_logger(), "finished");

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}

mrs_lib::Task<std::optional<std::shared_ptr<typename std_srvs::srv::SetBool::Response>>>
call_indirect(mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>& client, std::shared_ptr<std_srvs::srv::SetBool::Request> request, size_t depth)
{
  if (depth == 0)
  {
    co_return co_await client.callAwaitable(request);
  } else
  {
    co_return co_await call_indirect(client, std::move(request), depth - 1);
  }
}


TEST_F(Test, CoroCall)
{
  // Coroutine based callbacks should handle waiting on single threaded executor
  initialize<rclcpp::executors::SingleThreadedExecutor>(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service server ---------------- |

  const auto callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
      "/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), callback_group_);

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "service1");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  std::atomic<bool> completed = false;
  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() -> mrs_lib::Task<> {
    tim->cancel(); // just a one-shot timer

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    {
      request->data = true;

      auto opt_response = co_await call_indirect(client1, request, 5);

      EXPECT_TRUE(opt_response.has_value());
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
    despin();
  };

  tim = node_->create_timer(0s, [test_fun]() -> void { mrs_lib::internal::start_task(test_fun); }, callback_group_);
  executor_->spin();

  ASSERT_TRUE(completed);
}

/* TEST_F(Test, test_bad_address) //{ */

TEST_F(Test, test_bad_address)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "random");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() {
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

      request->data = true;

      auto response = client1.callSync(request);

      EXPECT_FALSE(response.has_value());
    }

    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

      request->data = true;

      auto response = client1.callAsync(request);

      clock->sleep_for(2s);

      EXPECT_FALSE(response.has_value());
    }

    RCLCPP_INFO(node_->get_logger(), "finished");

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}

/* TEST_F(Test, test_get_service) //{ */

TEST_F(Test, test_get_service)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "/my_service");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() {
    tim->cancel();

    const std::string name = client.getServiceName();

    EXPECT_EQ(name, "/my_service");

    RCLCPP_INFO(node_->get_logger(), "finished");

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}

/* TEST_F(Test, test_is_service_ready) //{ */

TEST_F(Test, test_is_service_ready)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "/readiness_service");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() {
    tim->cancel();

    // no server is running — should not be ready
    EXPECT_FALSE(client.isServiceReady());

    // | ------------ start a service server and re-check ---------- |

    const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
        "/readiness_service", std::bind(&Test::callbackFailedService, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), svr_grp);

    // give the executor a moment to register the server
    node_->get_clock()->sleep_for(100ms);

    EXPECT_TRUE(client.isServiceReady());

    RCLCPP_INFO(node_->get_logger(), "finished");

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}

/* TEST_F(Test, test_wait_for_service) //{ */

TEST_F(Test, test_wait_for_service)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "/wait_service");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() {
    tim->cancel();

    // no server running — should time out
    EXPECT_FALSE(client.waitForService(200ms));

    // | ------------ start a server and wait again ---------------- |

    const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    const auto service_server = node_->create_service<std_srvs::srv::SetBool>(
        "/wait_service", std::bind(&Test::callbackFailedService, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), svr_grp);

    // should become available well within 2 seconds
    EXPECT_TRUE(client.waitForService(2s));

    RCLCPP_INFO(node_->get_logger(), "finished");

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}

/* TEST_F(Test, test_uninitialized_new_methods) //{ */

TEST_F(Test, test_uninitialized_new_methods)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  // default-constructed handler — all new methods must degrade gracefully
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client;

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]() {
    tim->cancel();

    EXPECT_EQ(client.getServiceName(), "");
    EXPECT_FALSE(client.isServiceReady());
    EXPECT_FALSE(client.waitForService(100ms));

    RCLCPP_INFO(node_->get_logger(), "finished");

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}
