#include <cmath>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/param_provider.h>

#include <rcpputils/filesystem_helper.hpp>

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

  rcpputils::fs::path test_resources_path{TEST_RESOURCES_DIRECTORY};
};

/* TEST_F(Test, param_provider_declare) //{ */

TEST_F(Test, param_provider_declare) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  auto pp = mrs_lib::ParamProvider(node_, true);

  const auto name = "test_int";
  int test_int = -666;

  EXPECT_FALSE(pp.getParam(name, test_int));
  EXPECT_EQ(test_int, -666);
  EXPECT_FALSE(node_->has_parameter(name));

  int init_value = 1;
  EXPECT_TRUE(pp.declareParam(name, init_value));
  EXPECT_TRUE(node_->has_parameter(name));

  EXPECT_FALSE(pp.setParam(name, 13.4f));
  EXPECT_TRUE(pp.getParam(name, test_int));
  EXPECT_EQ(test_int, init_value);

  despin();

  clock->sleep_for(1s);
}

//}

/* TEST_F(Test, param_provider_load) //{ */

TEST_F(Test, param_provider_load) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  auto param_provider = mrs_lib::ParamProvider(node_, false);

  bool test_bool = false;
  EXPECT_FALSE(param_provider.getParam("test_bool", test_bool));

  EXPECT_FALSE(param_provider.addYamlFile(""));
  EXPECT_FALSE(param_provider.addYamlFile(test_resources_path.string() + "/nonexistent_config.yaml"));
  EXPECT_TRUE(param_provider.addYamlFile(test_resources_path.string() + "/test_config.yaml"));

  EXPECT_TRUE(param_provider.getParam("param_provider/test_bool", test_bool));
  EXPECT_EQ(test_bool, true);

  std::string test_str = "none";
  EXPECT_TRUE(param_provider.getParam("param_provider/test_str", test_str));
  EXPECT_EQ(test_str, "some_str");

  int test_int = 42;
  EXPECT_TRUE(param_provider.getParam("param_provider/test_int", test_int));
  EXPECT_EQ(test_int, 666);

  double test_double = 42.424242;
  EXPECT_TRUE(param_provider.getParam("param_provider/test_double", test_double));
  EXPECT_DOUBLE_EQ(test_double, 666.666);

  std::vector<bool> test_bool_array;
  EXPECT_TRUE(param_provider.getParam("param_provider/test_bool_array", test_bool_array));

  std::vector<std::string> test_str_array;
  EXPECT_TRUE(param_provider.getParam("param_provider/test_str_array", test_str_array));

  std::vector<long int> test_int_array;
  EXPECT_TRUE(param_provider.getParam("param_provider/test_int_array", test_int_array));

  std::vector<double> test_double_array;
  EXPECT_TRUE(param_provider.getParam("param_provider/test_double_array", test_double_array));

  despin();

  clock->sleep_for(1s);
}

//}

/* TEST_F(Test, param_provider_set) //{ */

TEST_F(Test, param_provider_set) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false));

  auto clock = node_->get_clock();

  auto pp = mrs_lib::ParamProvider(node_, true);

  const auto name_raw = "test_int";
  const auto name = pp.resolveName(name_raw);
  mrs_lib::ParamProvider::range_t<int> range = {.minimum = -1, .maximum = 100};
  EXPECT_FALSE(pp.declareParam<int>(name, {.reconfigurable = true, .default_value = -100, .range = range}));
  int init_value = 1;
  EXPECT_TRUE(pp.declareParam<int>(name, {.reconfigurable = true, .default_value = init_value, .range = range}));

  int test_int = -666;
  EXPECT_FALSE(pp.setParam(name_raw, 666));
  EXPECT_TRUE(pp.getParam(name_raw, test_int));
  EXPECT_EQ(test_int, init_value);

  EXPECT_TRUE(pp.setParam(name_raw, -1));
  EXPECT_TRUE(pp.getParam(name_raw, test_int));
  EXPECT_EQ(test_int, -1);

  EXPECT_TRUE(pp.setParam(name_raw, 66));
  EXPECT_TRUE(pp.getParam(name_raw, test_int));
  EXPECT_EQ(test_int, 66);

  despin();

  clock->sleep_for(1s);
}

//}
