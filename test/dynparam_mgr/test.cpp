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

  template <typename T>
  void test_with_type(mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value, const std::vector<T>& test_values);

  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
};

template <typename T>
void Test::test_with_type(mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value, const std::vector<T>& test_values)
{
  T test_var = init_value;
  RCLCPP_INFO(node_->get_logger(), "trying to register test_var without default value");
  EXPECT_FALSE(dynparam_mgr.register_param(param_name, &test_var));

  // set the value of the test_var parameter
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "setting " << param_name);
    const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter(param_name, test_values.front()));
    EXPECT_TRUE(result.successful);
  }

  // because the registering above failed, the value of the test_var variable should remain unchanged even though the parameter is now defined and is true
  EXPECT_EQ(test_var, init_value);

  // register test_var again - this time successfully because the parameter already has a value
  RCLCPP_INFO_STREAM(node_->get_logger(), "registering " << param_name);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_var));
  // now the value should be loaded from ROS, so the value of the test_var variable should be the same as that of the ROS parameter - true
  EXPECT_EQ(test_var, test_values.front());

  // now if the ROS parameter value is changed, the value of the variable should change as well
  for (const auto& test_val : test_values)
  {
    // change the value of the test_var parameter
    const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter(param_name, test_val));
    // check that the setting was successful
    EXPECT_TRUE(result.successful);
    // now the variable should be changed
    EXPECT_EQ(test_var, test_val);
  }
}

/* TEST_F(Test, dynparam_mgr_init) //{ */

TEST_F(Test, dynparam_mgr_init) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx, node_->get_name());

  test_with_type<bool>(dynparam_mgr, "test_bool", false, {true, false});

  test_with_type<int>(dynparam_mgr, "test_int", 0, {-1, 1, 666});

  test_with_type<int64_t>(dynparam_mgr, "test_int64", 0, {-1, 1, 666});

  test_with_type<float>(dynparam_mgr, "test_float", 0.0f, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});

  test_with_type<double>(dynparam_mgr, "test_double", 0.0, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});

  test_with_type<std::string>(dynparam_mgr, "test_string", "", {"asdf", "gbleasdgelasdagdds", "554"});

  test_with_type<std::vector<uint8_t>>(dynparam_mgr, "test_bytearr", {}, {{123, 1, 2}, {255}, {}});

  test_with_type<std::vector<bool>>(dynparam_mgr, "test_boolarr", {}, {{true, false, true}, {false}, {}});

  /* test_with_type<std::vector<int>>(dynparam_mgr, "test_intarr", {}, {{1, 2, 3}, {-1}, {}}); */

  test_with_type<std::vector<int64_t>>(dynparam_mgr, "test_int64arr", {}, {{1, 2, 3}, {-1}, {}});

  /* test_with_type<std::vector<float>>(dynparam_mgr, "test_floatarr", {}, {{1.0f, 2.0f, 3.0f}, {-1.0f}, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}, {}}); */

  test_with_type<std::vector<double>>(dynparam_mgr, "test_doublearr", {}, {{1.0, 2.0, 3.0}, {-1.0}, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, {}});

  test_with_type<std::vector<std::string>>(dynparam_mgr, "test_stringarr", {}, {{"asdf", "gbleasdgelasdagdds", "554"}, {}, {"blebleble"}});

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}
