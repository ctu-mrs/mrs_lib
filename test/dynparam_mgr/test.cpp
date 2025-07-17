#include <gtest/gtest.h>

#include <mrs_lib/dynparam_mgr.h>

using namespace std::chrono_literals;
using namespace std::string_literals;

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

template <typename T>
struct test_param_t
{
  std::string name;
  T init_value;
  T yaml_value;
  std::vector<T> test_values;
  T param_variable = {}; // persistent variable
};

const auto test_parameters_defaults = std::tuple
{
  test_param_t<bool>{"test_bool", false, true, {true, false}},
  test_param_t<int>{"test_int", 0, 333, {-1, 1, 666}},
  test_param_t<int64_t>{"test_int64", 0, 334, {-1, 1, 666}},
  test_param_t<float>{"test_float", 0.0f, 335, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}},
  test_param_t<double>{"test_double", 0.0, 336, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}},
  test_param_t<std::string>{"test_string", "", "some_str", {"asdf", "gbleasdgelasdagdds", "554"}},
  test_param_t<std::vector<uint8_t>>{"test_bytearr", {}, {0, 1}, {{123, 1, 2}, {255}, {}}},
  test_param_t<std::vector<bool>>{"test_boolarr", {}, {true, false}, {{true, false, true}, {false}, {}}},
  test_param_t<std::vector<int64_t>>{"test_int64arr", {}, {5, 6, 7, 8}, {{1, 2, 3}, {-1}, {}}},
  test_param_t<std::vector<double>>{"test_doublearr", {}, {0.1, 1.2, 2.3, 3.4}, {{1.0, 2.0, 3.0}, {-1.0}, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, {}}},
  test_param_t<std::vector<std::string>>{"test_stringarr", {}, {"this", "should", "load"}, {{"asdf", "gbleasdgelasdagdds", "554"}, {}, {"blebleble"}}},
};

/* TEST_F(Test, dynparam_mgr_existing_param) //{ */

/* set_param_type(mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value, const std::vector<T>& test_values) //{ */

template <typename T>
void set_param_type(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const T& init_value, const std::vector<T>& test_values)
{
  test_var = init_value;

  // register test_var
  RCLCPP_INFO_STREAM(node->get_logger(), "registering " << param_name);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_var));
  // now the value should be loaded from ROS, so the value of the test_var variable should be the same as that of the ROS parameter - true
  EXPECT_EQ(test_var, test_values.front());

  // now if the ROS parameter value is changed, the value of the variable should change as well
  for (const auto& test_val : test_values)
  {
    // change the value of the test_var parameter
    const auto result = dynparam_mgr.get_param_provider().setParam(param_name, test_val);
    // check that the setting was successful
    EXPECT_TRUE(result);
    // now the variable should be changed
    EXPECT_EQ(test_var, test_val);
  }
}

//}

TEST_F(Test, dynparam_mgr_existing_param) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto test_parameters = test_parameters_defaults;

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::apply
  (
    [&node_options](const auto& ... tupleArgs)
    {
      ((
        node_options.append_parameter_override(tupleArgs.name, tupleArgs.test_values.front())
        , ...));
    }, test_parameters
  );

  RCLCPP_INFO(node_->get_logger(), "defining dynparam_mgr_existing_param");
  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("dynparam_mgr_existing_param", "", node_options);

  std::mutex mtx;
  RCLCPP_INFO(node2->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node2, mtx);

  std::apply
  (
    [node2, &dynparam_mgr](auto& ... tupleArgs)
    {
      ((set_param_type(node2, dynparam_mgr, tupleArgs.param_variable, tupleArgs.name, tupleArgs.init_value, tupleArgs.test_values), ...));
    }, test_parameters
  );

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_nonexisting_param) //{ */

template <typename T>
void dynparam_mgr_nonexisting_param(mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const T& init_value)
{
  EXPECT_FALSE(dynparam_mgr.register_param(param_name, &test_var));
  EXPECT_EQ(test_var, init_value);
}

TEST_F(Test, dynparam_mgr_nonexisting_param) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto test_parameters = test_parameters_defaults;

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx);

  std::apply
  (
    [&dynparam_mgr](auto& ... tupleArgs)
    {
      ((dynparam_mgr_nonexisting_param(dynparam_mgr, tupleArgs.param_variable, tupleArgs.name, tupleArgs.init_value), ...));
    }, test_parameters
  );

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_default_value) //{ */

/* default_value_test(mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value) //{ */

template <typename T>
void default_value_test(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const T& init_value, const T& default_value)
{
  test_var = init_value;

  // register test_var
  RCLCPP_INFO_STREAM(node->get_logger(), "registering " << param_name);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_var, default_value));
  // the parameter shouldnot be available anywhere, so it should be equal to the default value
  EXPECT_EQ(test_var, default_value);
}

//}

TEST_F(Test, dynparam_mgr_default_value) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto test_parameters = test_parameters_defaults;

  RCLCPP_INFO(node_->get_logger(), "defining dynparam_mgr_default_value");
  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("dynparam_mgr_default_value", "");

  std::mutex mtx;
  RCLCPP_INFO(node2->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node2, mtx);

  std::apply
  (
    [node2, &dynparam_mgr](auto& ... tupleArgs)
    {
    // the static cast bullshit is a fix for a std::vector<bool>
      ((default_value_test(node2, dynparam_mgr, tupleArgs.param_variable, tupleArgs.name, tupleArgs.init_value, static_cast<decltype(tupleArgs.init_value)>(tupleArgs.test_values.at(1))), ...));
    }, test_parameters
  );

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_update_params) //{ */

/* update_to_ros_test(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value, const std::vector<T>& test_values) //{ */

template <typename T>
void update_to_ros_test(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const T& var_init_value, const T& first_value, const std::vector<T>& test_values)
{
  test_var = var_init_value;

  // register test_var
  RCLCPP_INFO_STREAM(node->get_logger(), "registering " << param_name);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_var));
  // now the value should be loaded from ROS, so the value of the test_var variable should be the same as that of the ROS parameter - true
  EXPECT_EQ(test_var, first_value);

  // now if the ROS parameter value is changed, the value of the variable should change as well
  for (const auto& test_val : test_values)
  {
    // change the value of the variable
    test_var = test_val;
    // update the value to ROS
    RCLCPP_INFO_STREAM(node->get_logger(), "updating parameters to ROS");
    const auto result = dynparam_mgr.update_to_ros();
    EXPECT_TRUE(result.successful);
    if (!result.successful)
      RCLCPP_ERROR_STREAM(node->get_logger(), result.reason);

    // value of the ROS parameter should now be changed
    RCLCPP_INFO_STREAM(node->get_logger(), "getting the ROS value");
    const auto resolved_name = dynparam_mgr.get_param_provider().resolveName(param_name);
    T ros_param_val;
    dynparam_mgr.get_param_provider().getParam(resolved_name, ros_param_val, {.use_yaml = false});
    EXPECT_EQ(ros_param_val, test_val);
    // value of the variable should be what it was set to
    EXPECT_EQ(test_var, test_val);
  }
}

//}

TEST_F(Test, dynparam_mgr_update_params) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto test_parameters = test_parameters_defaults;

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::apply
  (
    [&node_options](const auto& ... tupleArgs)
    {
      ((
        node_options.append_parameter_override(tupleArgs.name, tupleArgs.test_values.front())
        , ...));
    }, test_parameters
  );

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("dynparam_mgr_update_params", "", node_options);

  std::mutex mtx;
  RCLCPP_INFO(node2->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node2, mtx);

  std::apply
  (
    [node2, &dynparam_mgr](auto& ... tupleArgs)
    {
    // the static cast bullshit is a fix for a std::vector<bool>
      ((update_to_ros_test(node2, dynparam_mgr, tupleArgs.param_variable, tupleArgs.name, tupleArgs.init_value, static_cast<decltype(tupleArgs.init_value)>(tupleArgs.test_values.front()), tupleArgs.test_values), ...));
    }, test_parameters
  );

  RCLCPP_INFO(node2->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_subnode) //{ */

TEST_F(Test, dynparam_mgr_subnode) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto test_parameters = test_parameters_defaults;

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("dynparam_mgr_subnode");

  /* std::apply */
  /* ( */
  /*   [&node2](const auto& ... tupleArgs) */
  /*   { */
  /*     (( */
  /*       node2->undeclare_parameter(tupleArgs.name) */
  /*       , ...)); */
  /*   }, test_parameters */
  /* ); */

  RCLCPP_INFO(node2->get_logger(), "defining subnode");
  auto subnode = node2->create_sub_node("subnode");

  std::mutex mtx;
  RCLCPP_INFO(subnode->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(subnode, mtx);
  // add the YAML files with the default values of the parameters
  EXPECT_TRUE(dynparam_mgr.get_param_provider().addYamlFile(test_resources_path.string() + "/test_config.yaml"));

  std::apply
  (
    [subnode, &dynparam_mgr](auto& ... tupleArgs)
    {
      ((update_to_ros_test(subnode, dynparam_mgr, tupleArgs.param_variable, tupleArgs.name, tupleArgs.init_value, tupleArgs.yaml_value, tupleArgs.test_values), ...));
    }, test_parameters
  );

  RCLCPP_INFO(node2->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_callback) //{ */

struct CallbackTesterBase {};

template <typename T>
struct CallbackTester : CallbackTesterBase
{
  std::string tested_param_name;
  rclcpp::Node::SharedPtr node;
  T expected_param_value = {};
  int valid_callbacks = 0;

  CallbackTester(const std::string& tested_param_name, const rclcpp::Node::SharedPtr& node)
    : tested_param_name(tested_param_name), node(node)
  {}

  void callback(const std::string& param_name, const T& new_value)
  {
    if (param_name != tested_param_name)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Parameter name \"" << param_name << "\" doesn't match the expected name \"" << tested_param_name << "\".");
      return;
    }

    if (new_value != expected_param_value)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "New value of parameter \"" << param_name << "\" doesn't match the expected.");
      return;
    }

    valid_callbacks++;
    RCLCPP_INFO_STREAM(node->get_logger(), "Got new value for parameter \"" << param_name << "\".");
  }
};

template <typename T>
std::shared_ptr<CallbackTesterBase> test_callback(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const std::vector<T>& test_values)
{
  T test_var;

  auto tester_ptr = std::make_shared<CallbackTester<T>>(param_name, node);

  const mrs_lib::DynparamMgr::update_cbk_t<T> cbk = std::bind(&CallbackTester<T>::callback, tester_ptr.get(), param_name, std::placeholders::_1);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_var, cbk));

  // now if the ROS parameter value is changed, the value of the variable should change as well
  for (const auto& test_val : test_values)
  {
    tester_ptr->expected_param_value = test_val;
    EXPECT_TRUE(dynparam_mgr.get_param_provider().setParam(param_name, test_val));
  }
  EXPECT_EQ(tester_ptr->valid_callbacks, test_values.size());

  // the pointer has to be returned and kept by the caller so that the callback is still valid
  return tester_ptr;
}

TEST_F(Test, dynparam_mgr_callback)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto test_parameters = test_parameters_defaults;

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx);
  // add the YAML files with the default values of the parameters
  EXPECT_TRUE(dynparam_mgr.get_param_provider().addYamlFile(test_resources_path.string() + "/test_config.yaml"));

  // a vector to hold the objects created within the test_callback so that they don't go out of scope
  // while being used
  std::vector<std::shared_ptr<CallbackTesterBase>> cbk_tester_ptrs;
  std::apply
  (
    [this, &dynparam_mgr, &cbk_tester_ptrs](const auto& ... tupleArgs)
    {
      ((cbk_tester_ptrs.push_back(test_callback(node_, dynparam_mgr, tupleArgs.name, tupleArgs.test_values)), ...));
    }, test_parameters
  );

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

template <typename T>
using range_t = mrs_lib::DynparamMgr::range_t<T>;

/* TEST_F(Test, dynparam_mgr_ranges) //{ */

template <typename T>
void test_ranges(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const T& minimum, const T& maximum, const std::string& param_name, const std::vector<T>& test_values)
{
  // all the parameters should be available in the YAML file
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_var, {minimum, maximum}));

  // now if the ROS parameter value is changed, the value of the variable should change as well
  for (const auto& test_val : test_values)
  {
    const T prev_val = test_var;
    const auto parameter_set_result = dynparam_mgr.get_param_provider().setParam(param_name, test_val);
    if (test_val >= minimum && test_val <= maximum)
    {
      if (!parameter_set_result)
        RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to set a new value " << test_val << " of parameter \"" << param_name << "\" with range [" << minimum << ", " << maximum << "].");
      EXPECT_TRUE(parameter_set_result);
      EXPECT_EQ(test_var, test_val);
    }
    else
    {
      if (parameter_set_result)
        RCLCPP_ERROR_STREAM(node->get_logger(), "Succeeded to set a new value " << test_val << " of parameter \"" << param_name << "\" with range [" << minimum << ", " << maximum << "].");
      EXPECT_FALSE(parameter_set_result);
      EXPECT_EQ(test_var, prev_val);
    }
  }
}

TEST_F(Test, dynparam_mgr_ranges)
{

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx);
  // add the YAML files with the default values of the parameters
  EXPECT_TRUE(dynparam_mgr.get_param_provider().addYamlFile(test_resources_path.string() + "/test_config.yaml"));

  int test_int;
  int64_t test_int64;
  float test_float;
  double test_double;

  test_ranges(node_, dynparam_mgr, test_int, -3, 666, "test_int", {-1, 1, 666, 667});
  test_ranges(node_, dynparam_mgr, test_int64, -3l, 666l, "test_int64", {-1l, 1l, 666l, 667l});
  test_ranges(node_, dynparam_mgr, test_float, -3.0f, 666.0f, "test_float", {-1.0f, 1.0f, 666.0f, 667.0f, 1e-3f});
  test_ranges(node_, dynparam_mgr, test_double, -3.0, 666.0, "test_double", {-1.0, 1.0, 666.0, 667.0, 1e-3});
  // for some reason, negative and positive infinity can always be set even if a parameter range is specified... it's another ROS2 bug - yay!
  /* test_ranges(node_, dynparam_mgr, test_float, -3.0f, 666.0f, "test_float", {-1.0f, 1.0f, 666.0f, 667.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}); */
  /* test_ranges(node_, dynparam_mgr, test_double, -3.0, 666.0, "test_double", {-1.0, 1.0, 666.0, 667.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}); */

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_loaded_successfully) //{ */

TEST_F(Test, dynparam_mgr_loaded_successfully) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));
  RCLCPP_INFO(node_->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx);
  EXPECT_TRUE(dynparam_mgr.get_param_provider().addYamlFile(test_resources_path.string() + "/test_config.yaml"));

  int test_int;
  const std::string param_name = "test_int";

  RCLCPP_INFO_STREAM(node_->get_logger(), "registering " << param_name);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_int));
  EXPECT_TRUE(dynparam_mgr.loaded_successfully());

  const std::string nonexistent_param_name = "doesnt_exist";
  RCLCPP_INFO_STREAM(node_->get_logger(), "registering " << nonexistent_param_name);
  EXPECT_FALSE(dynparam_mgr.register_param(nonexistent_param_name, &test_int));
  EXPECT_FALSE(dynparam_mgr.loaded_successfully());

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

