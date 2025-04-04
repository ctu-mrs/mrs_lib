#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/service_server_handler.h>

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

      node_ = std::make_shared<rclcpp::Node>("test_service_server_handler", node_options);

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

  const auto sshandler = mrs_lib::ServiceServerHandler<std_srvs::srv::SetBool>(node_, "/service1", std::bind(&Test::callbackService, this, std::placeholders::_1, std::placeholders::_2));

  // | ----------------- create a service client ---------------- |

  // the service client has to be in a different callback group than the timer!
  // otherwise, the callback will never be called, because the default callback
  // group is MutuallyExclusive
  const auto svr_grp = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto client = node_->create_client<std_srvs::srv::SetBool>("service1", rclcpp::ServicesQoS(), svr_grp);

  RCLCPP_INFO(node_->get_logger(), "initialized");

  rclcpp::TimerBase::SharedPtr tim;

  const auto test_fun = [&]()
  {
    tim->cancel(); // just a one-shot timer

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    {
      request->data = true;

      auto resp_fut = client->async_send_request(request);
      EXPECT_TRUE(resp_fut.valid());

      while (rclcpp::ok()) {

        RCLCPP_INFO(node_->get_logger(), "waiting for the future response");

        if (resp_fut.wait_for(1s) == std::future_status::ready)
          break;
      }

      const auto& resp = resp_fut.get();
      EXPECT_TRUE(resp->success);
      EXPECT_EQ(resp->message, "set");
    }

    RCLCPP_INFO(node_->get_logger(), "finished");

    despin();
  };

  tim = node_->create_timer(0s, test_fun);
  executor_->spin();
}

//}
