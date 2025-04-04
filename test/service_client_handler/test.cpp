#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/service_client_handler.h>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

class Test : public ::testing::Test {

  public:
    void callbackService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void callbackRepeatedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void callbackFailedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  protected:
    /* SetUpTestCase() //{ */

    static void SetUpTestCase() {
      rclcpp::init(0, nullptr);
    }

    //}

    /* TearDownTestCase() //{ */

    static void TearDownTestCase() {
      rclcpp::shutdown();
    }

    //}

    /* initialize() //{ */

    void initialize(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) {

      node_ = std::make_shared<rclcpp::Node>("test_service_client_handler", node_options);

      executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      executor_->add_node(node_);
    }

    //}

    /* despin() //{ */

    void despin() {
      executor_->cancel();
    }

    //}

    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
};

/* callbackService() //{ */

void Test::callbackService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  response->success = true;

  node_->get_clock()->sleep_for(1s);

  if (request->data) {
    response->message = "set";
  } else {
    response->message = "unset";
  }
}

//}

/* callbackFailedService() //{ */

void Test::callbackFailedService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  response->success = true;

  if (request->data) {
    response->message = "set";
  } else {
    response->message = "unset";
  }
}

//}

/* TEST_F(Test, test_call) //{ */

TEST_F(Test, test_call) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service server ---------------- |

  // the service server has to be in a different callback group than the timer!
  // otherwise, the callback will never be called, because the default callback
  // group is MutuallyExclusive
  const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto service_server =
    node_->create_service<std_srvs::srv::SetBool>("/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), svr_grp);

  // | ----------------- create a service client ---------------- |

  // the ServiceClientHandler by default creates its own MutuallyExclusive callback group
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "service1");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]()
  {
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

TEST_F(Test, asynctest_call) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service server ---------------- |

  // the service server has to be in a different callback group than the timer!
  // otherwise, the callback will never be called, because the default callback
  // group is MutuallyExclusive
  const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto service_server =
    node_->create_service<std_srvs::srv::SetBool>("/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2), rclcpp::ServicesQoS(), svr_grp);

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "service1");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]()
  {
    tim->cancel(); // just a one-shot timer

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    {
      request->data = true;

      auto opt_response = client1.callAsync(request);

      ASSERT_TRUE(opt_response.has_value());

      auto response = opt_response.value();

      while (rclcpp::ok()) {

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

/* TEST_F(Test, test_bad_address) //{ */

TEST_F(Test, test_bad_address) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "random");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]()
  {
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
