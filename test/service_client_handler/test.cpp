#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/service_client_handler.h>

#include <std_srvs/srv/set_bool.hpp>

#include <thread>

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

      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);

      finished_future_ = finished_promise_.get_future();

      main_thread_ = std::thread(&Test::spin, this);
    }

    //}

    /* spin() //{ */

    void spin() {

      RCLCPP_INFO(node_->get_logger(), "starting spinning");

      executor_->spin();

      RCLCPP_INFO(node_->get_logger(), "stopped spinning");
    }

    //}

    /* despin() //{ */

    void despin() {
      executor_->cancel();

      main_thread_.join();
    }

    //}

    rclcpp::Node::SharedPtr                              node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

    std::thread main_thread_;

    std::promise<bool> finished_promise_;
    std::future<bool>  finished_future_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_1_;

    int repeated_call = 0;
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

  service_server_1_ =
    node_->create_service<std_srvs::srv::SetBool>("/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2));

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "service1");

  RCLCPP_INFO(node_->get_logger(), "initialized");

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

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();

  clock->sleep_for(1s);
}

//}

/* TEST_F(Test, asynctest_call) //{ */

TEST_F(Test, asynctest_call) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service server ---------------- |

  service_server_1_ =
    node_->create_service<std_srvs::srv::SetBool>("/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2));

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "service1");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

  {
    request->data = true;

    auto opt_response = client1.callAsync(request);

    ASSERT_TRUE(opt_response.has_value());

    auto response = opt_response.value();

    while (rclcpp::ok()) {

      RCLCPP_INFO(node_->get_logger(), "waiting for the future response");

      if (response.valid()) {
        RCLCPP_INFO(node_->get_logger(), "future ready");
        break;
      }

      clock->sleep_for(0.1s);
    }

    EXPECT_TRUE(response.get()->success);
    EXPECT_EQ(response.get()->message, "set");
  }

  RCLCPP_INFO(node_->get_logger(), "finished");

  despin();

  clock->sleep_for(1s);
}

//}

/* TEST_F(Test, test_bad_address) //{ */

TEST_F(Test, test_bad_address) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  // | ----------------- create a service client ---------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> client1 = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "random");

  RCLCPP_INFO(node_->get_logger(), "initialized");

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

  clock->sleep_for(1s);
}

//}
