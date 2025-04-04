#include <gtest/gtest.h>

#include <mrs_lib/dynamic_params.h>

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

    node_ = std::make_shared<rclcpp::Node>("test_dynparam_mgr", node_options);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

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

  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
};

/* TEST_F(Test, dynparam_mgr_init) //{ */

TEST_F(Test, dynparam_mgr_init) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "defining ParamProvider");
  mrs_lib::ParamProvider pp(node_, node_->get_name());

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx, node_->get_name());

  bool test_bool;
  RCLCPP_INFO(node_->get_logger(), "trying to register test_bool without default value");
  EXPECT_FALSE(dynparam_mgr.register_param("test_bool", &test_bool));

  // set the value of the test_bool parameter
  {
    RCLCPP_INFO(node_->get_logger(), "setting test_bool");
    const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter("test_bool", true));
    RCLCPP_INFO_STREAM(node_->get_logger(), result.reason);
    EXPECT_TRUE(result.successful);
  }

  // because the registering above failed, the value of the test_bool variable should remain unchanged even though the parameter is now defined and is true
  EXPECT_FALSE(test_bool);

  // register test_bool again - this time successfully because the parameter already has a value
  RCLCPP_INFO(node_->get_logger(), "registering test_bool");
  EXPECT_TRUE(dynparam_mgr.register_param("test_bool", &test_bool));
  // now the value should be loaded from ROS, so the value of the test_bool variable should be the same as that of the ROS parameter - true
  EXPECT_TRUE(test_bool);

  // now if the ROS parameter value is changed, the value of the variable should change as well
  {
    // change the value of the test_bool parameter
    const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter("test_bool", false));
    // check that the setting was successful
    RCLCPP_INFO_STREAM(node_->get_logger(), result.reason);
    EXPECT_TRUE(result.successful);
  }
  // now the variable should be false
  EXPECT_FALSE(test_bool);

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}
