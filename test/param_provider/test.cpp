#include <cmath>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/param_provider.h>

#include <thread>
#include <vector>

using namespace std::chrono_literals;

class Test : public ::testing::Test {

public:
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

    node_ = std::make_shared<rclcpp::Node>("test_param_provider", node_options);

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
};

/* TEST_F(Test, param_provider_load) //{ */

TEST_F(Test, param_provider_load) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  auto param_provider = mrs_lib::ParamProvider(node_, node_->get_name(), false);

  bool test_bool = false;
  EXPECT_FALSE(param_provider.getParam("test_bool", test_bool));

  EXPECT_FALSE(param_provider.addYamlFile(""));
  EXPECT_FALSE(param_provider.addYamlFile("./custom_test_config.yaml"));
  EXPECT_TRUE(param_provider.addYamlFile("/tmp/mrs_lib/test/param_provider/test_config.yaml"));

  EXPECT_TRUE(param_provider.getParam("test_bool", test_bool));
  EXPECT_EQ(test_bool, true);

  std::string test_str = "none";
  EXPECT_TRUE(param_provider.getParam("test_str", test_str));
  EXPECT_EQ(test_str, "some_str");

  int test_int = 42;
  EXPECT_TRUE(param_provider.getParam("test_int", test_int));
  EXPECT_EQ(test_int, 666);
  
  double test_double = 42.424242;
  EXPECT_TRUE(param_provider.getParam("test_double", test_double));
  EXPECT_DOUBLE_EQ(test_double, 666.666);

  std::vector<bool> test_bool_array;
  EXPECT_FALSE(param_provider.getParam("test_bool_array", test_bool_array));

  despin();

  clock->sleep_for(1s);
}

//}
