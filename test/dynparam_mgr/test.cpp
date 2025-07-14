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

  template <typename T>
  void set_param_type(mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value, const std::vector<T>& test_values);

  template <typename T>
  void update_params_type(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const T& init_value, const std::vector<T>& test_values) const;

  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
};

/* set_param_type(mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value, const std::vector<T>& test_values) //{ */

template <typename T>
void Test::set_param_type(mrs_lib::DynparamMgr& dynparam_mgr, const std::string& param_name, const T& init_value, const std::vector<T>& test_values)
{
  T test_var = init_value;
  RCLCPP_INFO_STREAM(node_->get_logger(), "trying to register " << param_name << " without default value");
  EXPECT_FALSE(dynparam_mgr.register_param(param_name, &test_var));

  // set the value of the test_var parameter
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "setting " << param_name);
    const auto result = dynparam_mgr.get_param_provider().setParam(param_name, test_values.front());
    EXPECT_TRUE(result);
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
    const auto result = dynparam_mgr.get_param_provider().setParam(param_name, test_val);
    // check that the setting was successful
    EXPECT_TRUE(result);
    // now the variable should be changed
    EXPECT_EQ(test_var, test_val);
  }
}

//}

/* update_params_type(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const T& init_value, const std::vector<T>& test_values) //{ */

template <typename T>
void Test::update_params_type(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const T& init_value, const std::vector<T>& test_values) const
{
  test_var = init_value;
  RCLCPP_INFO_STREAM(node->get_logger(), "trying to register " << param_name << " without default value");
  EXPECT_FALSE(dynparam_mgr.register_param(param_name, &test_var));

  // set the value of the test_var parameter
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "setting " << param_name);
    const auto result = dynparam_mgr.get_param_provider().setParam(param_name, test_values.front());
    EXPECT_TRUE(result);
  }

  // because the registering above failed, the value of the test_var variable should remain unchanged even though the parameter is now defined and is true
  EXPECT_EQ(test_var, init_value);

  // register test_var again - this time successfully because the parameter already has a value
  RCLCPP_INFO_STREAM(node->get_logger(), "registering " << param_name);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_var));
  // now the value should be loaded from ROS, so the value of the test_var variable should be the same as that of the ROS parameter - true
  EXPECT_EQ(test_var, test_values.front());

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
    T ros_param_val;
    dynparam_mgr.get_param_provider().getParam(param_name, ros_param_val);
    EXPECT_EQ(ros_param_val, test_val);
    // value of the variable should be what it was set to
    EXPECT_EQ(test_var, test_val);
  }
}

//}

/* TEST_F(Test, dynparam_mgr_set_param) //{ */

TEST_F(Test, dynparam_mgr_set_param) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx);

  set_param_type<bool>(dynparam_mgr, "test_bool", false, {true, false});

  set_param_type<int>(dynparam_mgr, "test_int", 0, {-1, 1, 666});

  set_param_type<int64_t>(dynparam_mgr, "test_int64", 0, {-1, 1, 666});

  set_param_type<float>(dynparam_mgr, "test_float", 0.0f, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});

  set_param_type<double>(dynparam_mgr, "test_double", 0.0, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});

  set_param_type<std::string>(dynparam_mgr, "test_string", "", {"asdf", "gbleasdgelasdagdds", "554"});

  set_param_type<std::vector<uint8_t>>(dynparam_mgr, "test_bytearr", {}, {{123, 1, 2}, {255}, {}});

  set_param_type<std::vector<bool>>(dynparam_mgr, "test_boolarr", {}, {{true, false, true}, {false}, {}});

  /* set_param_type<std::vector<int>>(dynparam_mgr, "test_intarr", {}, {{1, 2, 3}, {-1}, {}}); */

  set_param_type<std::vector<int64_t>>(dynparam_mgr, "test_int64arr", {}, {{1, 2, 3}, {-1}, {}});

  /* set_param_type<std::vector<float>>(dynparam_mgr, "test_floatarr", {}, {{1.0f, 2.0f, 3.0f}, {-1.0f}, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}, {}}); */

  set_param_type<std::vector<double>>(dynparam_mgr, "test_doublearr", {}, {{1.0, 2.0, 3.0}, {-1.0}, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, {}});

  set_param_type<std::vector<std::string>>(dynparam_mgr, "test_stringarr", {}, {{"asdf", "gbleasdgelasdagdds", "554"}, {}, {"blebleble"}});

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_update_params) //{ */

TEST_F(Test, dynparam_mgr_update_params) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx);

  bool test_bool;
  int test_int;
  int64_t test_int64;
  float test_float;
  double test_double;
  std::string test_string;
  std::vector<uint8_t> test_bytearr;
  std::vector<bool> test_boolarr;
  /* std::vector<int> test_intarr; */
  std::vector<int64_t> test_int64arr;
  /* std::vector<float> test_floatarr; */
  std::vector<double> test_doublearr;
  std::vector<std::string> test_stringarr;

  update_params_type(node_, dynparam_mgr, test_bool, "test_bool", false, {true, false});
  update_params_type(node_, dynparam_mgr, test_int, "test_int", 0, {-1, 1, 666});
  update_params_type(node_, dynparam_mgr, test_int64, "test_int64", 0l, {-1l, 1l, 666l});
  update_params_type(node_, dynparam_mgr, test_float, "test_float", 0.0f, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});
  update_params_type(node_, dynparam_mgr, test_double, "test_double", 0.0, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
  update_params_type(node_, dynparam_mgr, test_string, "test_string", ""s, {"asdf"s, "gbleasdgelasdagdds"s, "554"s});
  update_params_type(node_, dynparam_mgr, test_bytearr, "test_bytearr", {}, {{123, 1, 2}, {255}, {}});
  update_params_type(node_, dynparam_mgr, test_boolarr, "test_boolarr", {}, {{true, false, true}, {false}, {}});
  /* update_params_type(node_, dynparam_mgr, test_intarr, "test_intarr", {}, {{1, 2, 3}, {-1}, {}}); */
  update_params_type(node_, dynparam_mgr, test_int64arr, "test_int64arr", {}, {{1, 2, 3}, {-1}, {}});
  /* update_params_type(node_, dynparam_mgr, test_floatarr, "test_floatarr", {}, {{1.0f, 2.0f, 3.0f}, {-1.0f}, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}, {}}); */
  update_params_type(node_, dynparam_mgr, test_doublearr, "test_doublearr", {}, {{1.0, 2.0, 3.0}, {-1.0}, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, {}});
  update_params_type(node_, dynparam_mgr, test_stringarr, "test_stringarr", {}, {{"asdf", "gbleasdgelasdagdds", "554"}, {}, {"blebleble"}});

  RCLCPP_INFO(node_->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_subnode) //{ */

TEST_F(Test, dynparam_mgr_subnode) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));
  auto subnode = node_->create_sub_node("subnode");

  auto clock = subnode->get_clock();

  RCLCPP_INFO(subnode->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(subnode->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(subnode, mtx);

  /* int declared_int_param; */
  /* EXPECT_TRUE(dynparam_mgr.get_param_provider().setParam("test_param", 666, true)); */
  /* dynparam_mgr.register_param("test_param", &declared_int_param); */
  /* RCLCPP_INFO_STREAM(subnode->get_logger(), "declared_param: " << declared_int_param); */
  /* int param_val; */
  /* dynparam_mgr.get_param_provider().getParam("test_param", param_val); */
  /* RCLCPP_INFO_STREAM(subnode->get_logger(), "param_val: " << param_val); */

  /* rclcpp::sleep_for(10000s); */

  bool test_bool;
  int test_int;
  int64_t test_int64;
  float test_float;
  double test_double;
  std::string test_string;
  std::vector<uint8_t> test_bytearr;
  std::vector<bool> test_boolarr;
  /* std::vector<int> test_intarr; */
  std::vector<int64_t> test_int64arr;
  /* std::vector<float> test_floatarr; */
  std::vector<double> test_doublearr;
  std::vector<std::string> test_stringarr;

  update_params_type(subnode, dynparam_mgr, test_bool, "test_bool", false, {true, false});
  update_params_type(subnode, dynparam_mgr, test_int, "test_int", 0, {-1, 1, 666});
  update_params_type(subnode, dynparam_mgr, test_int64, "test_int64", 0l, {-1l, 1l, 666l});
  update_params_type(subnode, dynparam_mgr, test_float, "test_float", 0.0f, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});
  update_params_type(subnode, dynparam_mgr, test_double, "test_double", 0.0, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
  update_params_type(subnode, dynparam_mgr, test_string, "test_string", ""s, {"asdf"s, "gbleasdgelasdagdds"s, "554"s});
  update_params_type(subnode, dynparam_mgr, test_bytearr, "test_bytearr", {}, {{123, 1, 2}, {255}, {}});
  update_params_type(subnode, dynparam_mgr, test_boolarr, "test_boolarr", {}, {{true, false, true}, {false}, {}});
  /* update_params_type(subnode, dynparam_mgr, test_intarr, "test_intarr", {}, {{1, 2, 3}, {-1}, {}}); */
  update_params_type(subnode, dynparam_mgr, test_int64arr, "test_int64arr", {}, {{1, 2, 3}, {-1}, {}});
  /* update_params_type(subnode, dynparam_mgr, test_floatarr, "test_floatarr", {}, {{1.0f, 2.0f, 3.0f}, {-1.0f}, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}, {}}); */
  update_params_type(subnode, dynparam_mgr, test_doublearr, "test_doublearr", {}, {{1.0, 2.0, 3.0}, {-1.0}, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, {}});
  update_params_type(subnode, dynparam_mgr, test_stringarr, "test_stringarr", {}, {{"asdf", "gbleasdgelasdagdds", "554"}, {}, {"blebleble"}});

  RCLCPP_INFO(subnode->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_loaded_successfully) //{ */

TEST_F(Test, dynparam_mgr_loaded_successfully) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));
  auto subnode = node_->create_sub_node("subnode");

  auto clock = subnode->get_clock();

  RCLCPP_INFO(subnode->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(subnode->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(subnode, mtx);

  bool test_bool;
  const std::string param_name = "test_bool";

  RCLCPP_INFO_STREAM(node_->get_logger(), "setting " << param_name);
  const auto result = dynparam_mgr.get_param_provider().setParam(param_name, false);
  EXPECT_TRUE(result);

  RCLCPP_INFO_STREAM(node_->get_logger(), "registering " << param_name);
  EXPECT_TRUE(dynparam_mgr.register_param(param_name, &test_bool));
  EXPECT_TRUE(dynparam_mgr.loaded_successfully());

  const std::string nonexistent_param_name = "doesnt_exist";
  RCLCPP_INFO_STREAM(node_->get_logger(), "registering " << nonexistent_param_name);
  EXPECT_TRUE(!dynparam_mgr.register_param(nonexistent_param_name, &test_bool));
  EXPECT_TRUE(!dynparam_mgr.loaded_successfully());

  RCLCPP_INFO(subnode->get_logger(), "finished");
  despin();
}

//}

/* TEST_F(Test, dynparam_mgr_callback) //{ */

template <typename T>
struct CallbackTester
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
std::shared_ptr<CallbackTester<T>> test_callback(rclcpp::Node::SharedPtr node, mrs_lib::DynparamMgr& dynparam_mgr, T& test_var, const std::string& param_name, const std::vector<T>& test_values)
{
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

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "defining ParamProvider");

  std::mutex mtx;
  RCLCPP_INFO(node_->get_logger(), "defining DynparamMgr");
  auto dynparam_mgr = mrs_lib::DynparamMgr(node_, mtx);
  // add the YAML files with the default values of the parameters
  EXPECT_TRUE(dynparam_mgr.get_param_provider().addYamlFile(test_resources_path.string() + "/test_config.yaml"));

  bool test_bool;
  int test_int;
  int64_t test_int64;
  float test_float;
  double test_double;
  std::string test_string;
  std::vector<uint8_t> test_bytearr;
  std::vector<bool> test_boolarr;
  /* std::vector<int> test_intarr; */
  std::vector<int64_t> test_int64arr;
  /* std::vector<float> test_floatarr; */
  std::vector<double> test_doublearr;
  std::vector<std::string> test_stringarr;

  // the returned pointers have to be stored to ensure that the callbacks are still valid
  const auto tester_bool = test_callback(node_, dynparam_mgr, test_bool, "test_bool", {true, false});
  const auto tester_int = test_callback(node_, dynparam_mgr, test_int, "test_int", {-1, 1, 666});
  const auto tester_int64 = test_callback(node_, dynparam_mgr, test_int64, "test_int64", {-1l, 1l, 666l});
  const auto tester_float = test_callback(node_, dynparam_mgr, test_float, "test_float", {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()});
  const auto tester_double = test_callback(node_, dynparam_mgr, test_double, "test_double", {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
  const auto tester_string = test_callback(node_, dynparam_mgr, test_string, "test_string", {"asdf"s, "gbleasdgelasdagdds"s, "554"s});
  const auto tester_bytearr = test_callback(node_, dynparam_mgr, test_bytearr, "test_bytearr", {{123, 1, 2}, {255}, {}});
  const auto tester_boolarr = test_callback(node_, dynparam_mgr, test_boolarr, "test_boolarr", {{true, false, true}, {false}, {}});
  /* const auto tester_intarr = test_callback(node_, dynparam_mgr, test_intarr, "test_intarr", {{1, 2, 3}, {-1}, {}}); */
  const auto tester_int64arr = test_callback(node_, dynparam_mgr, test_int64arr, "test_int64arr", {{1, 2, 3}, {-1}, {}});
  /* const auto tester_floatarr = test_callback(node_, dynparam_mgr, test_floatarr, "test_floatarr", {{1.0f, 2.0f, 3.0f}, {-1.0f}, {-1.0f, 1.0f, 666.0f, 1e-3f, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()}, {}}); */
  const auto tester_doublearr = test_callback(node_, dynparam_mgr, test_doublearr, "test_doublearr", {{1.0, 2.0, 3.0}, {-1.0}, {-1.0, 1.0, 666.0, 1e-3, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, {}});
  const auto tester_stringarr = test_callback(node_, dynparam_mgr, test_stringarr, "test_stringarr", {{"asdf", "gbleasdgelasdagdds", "554"}, {}, {"blebleble"}});

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
