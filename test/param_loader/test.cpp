#include <cmath>

#include <gtest/gtest.h>

#include <std_msgs/msg/int64.hpp>

#include <mrs_lib/param_loader.h>

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

/* TEST_F(Test, param_loader_load_from_file) //{ */

TEST_F(Test, param_loader_load_from_file) {

  initialize(rclcpp::NodeOptions().use_intra_process_comms(false).allow_undeclared_parameters(true));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "[%s]: initializing ParamLoader", node_->get_name());
  auto pl = mrs_lib::ParamLoader(node_, node_->get_name());

  RCLCPP_INFO(node_->get_logger(), "[%s]: testing ParamLoader", node_->get_name());

  bool test_bool = false;
  EXPECT_FALSE(pl.loadParam("test_bool", test_bool));
  pl.resetLoadedSuccessfully();

  EXPECT_FALSE(pl.addYamlFile(""));
  EXPECT_FALSE(pl.addYamlFile(test_resources_path.string() + "/custom_test_config.yaml"));
  pl.resetLoadedSuccessfully();

  EXPECT_TRUE(pl.addYamlFile(test_resources_path.string() + "/test_config.yaml"));

  std::vector<Eigen::MatrixXd> loaded_nd_matrix = pl.loadMatrixArray2("test_param_nd_matrix");
  EXPECT_TRUE(pl.loadedSuccessfully());

  if (pl.loadedSuccessfully()) {
    RCLCPP_INFO(node_->get_logger(), "[%s]: parameter loaded OK", node_->get_name());
    int it = 0;
    for (const auto& mat : loaded_nd_matrix) {
      std::cout << "matrix #" << it << std::endl;
      std::cout << mat << std::endl;
      it++;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: parameter loading failure", node_->get_name());
  }

  std::vector<Eigen::MatrixXd> vec;
  pl.resetUniques();
  pl.loadMatrixArray("test_param_nd_matrix", vec);
  EXPECT_TRUE(pl.loadedSuccessfully());

  pl.resetUniques();
  pl.loadMatrixArray("test_param_nd_matrix", vec, vec);
  EXPECT_TRUE(pl.loadedSuccessfully());

  pl.resetUniques();
  vec = pl.loadMatrixArray2("test_param_nd_matrix");
  EXPECT_TRUE(pl.loadedSuccessfully());

  pl.resetUniques();
  vec = pl.loadMatrixArray2("test_param_nd_matrix", vec);
  EXPECT_TRUE(pl.loadedSuccessfully());

  EXPECT_LT(fabs(loaded_nd_matrix.at(0)(0, 0) - (-5)), 1e-6);
  EXPECT_LT(fabs(loaded_nd_matrix.at(1)(2, 0) - (4)), 1e-6);

  std::cout << "Loading empty matrix" << std::endl;
  pl.resetUniques();
  Eigen::Matrix<double, 0, 0> mat = pl.loadMatrixStatic2<0, 0>("test_param_matrix_empty");
  std::cout << "Empty matrix:" << std::endl << mat;

  std::cout << "Testing different matrix loading" << std::endl;

  // | -------------------- loadMatrixStatic -------------------- |
  // reset the loadedSuccessfully flag back to true
  pl.resetLoadedSuccessfully();

  {
    Eigen::Matrix3d mat3d;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixStatic("test_param_matrix_3x3", mat3d));
    // check that the values are ordered properly
    EXPECT_LT(fabs(mat3d(0, 1) - (0.1)), 1e-6);
    EXPECT_LT(fabs(mat3d(1, 2) - (1.2)), 1e-6);
    EXPECT_LT(fabs(mat3d(2, 0) - (2.0)), 1e-6);
    EXPECT_LT(fabs(mat3d(2, 2) - (2.2)), 1e-6);

    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixStatic("test_param_matrix_3x3", mat3d, Eigen::Matrix3d::Identity()));
    pl.resetUniques();
    mat3d = pl.loadMatrixStatic2<3, 3, double>("test_param_matrix_3x3");
    pl.resetUniques();
    mat3d = pl.loadMatrixStatic2<3, 3, double>("test_param_matrix_nonexistent", Eigen::Matrix3d::Identity() * Eigen::Matrix3d::Ones());
    EXPECT_TRUE(pl.loadedSuccessfully());

    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixStatic("test_param_matrix_nonexistent", mat3d, Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(pl.loadedSuccessfully());

    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixStatic("test_param_matrix_empty", mat3d));
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  // | --------------------- loadMatrixKnown -------------------- |
  // reset the loadedSuccessfully flag back to true
  pl.resetLoadedSuccessfully();

  {
    Eigen::MatrixXd matxd;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixKnown("test_param_matrix_4x2", matxd, 4, 2));
    EXPECT_TRUE(pl.loadedSuccessfully());
    // check that the values are ordered properly
    EXPECT_LT(fabs(matxd(0, 1) - (0.1)), 1e-6);
    EXPECT_LT(fabs(matxd(2, 0) - (2.0)), 1e-6);
    EXPECT_LT(fabs(matxd(3, 1) - (3.1)), 1e-6);

    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixKnown("test_param_matrix_nonexistent", matxd, Eigen::MatrixXd::Identity(15, 16), 15, 16));
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixKnown2<double>("test_param_matrix_nonexistent", Eigen::MatrixXd::Identity(15, 16), 15, 16);
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixKnown2<double>("test_param_matrix_nonexistent", 15, 16);
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  // | -------------------- loadMatrixDynamic ------------------- |
  // reset the loadedSuccessfully flag back to true
  pl.resetLoadedSuccessfully();

  {
    Eigen::MatrixXd matxd;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixDynamic("test_param_matrix_3x3", matxd, 3, -1));
    EXPECT_TRUE(pl.loadedSuccessfully());
    EXPECT_EQ(matxd.rows(), 3);
    EXPECT_EQ(matxd.cols(), 3);
    // check that the values are ordered properly
    EXPECT_LT(fabs(matxd(0, 2) - (0.2)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 0) - (1.0)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 1) - (1.1)), 1e-6);
    EXPECT_LT(fabs(matxd(2, 0) - (2.0)), 1e-6);

    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixDynamic("test_param_matrix_3x3", matxd, -1, 3));
    EXPECT_TRUE(pl.loadedSuccessfully());
    EXPECT_EQ(matxd.rows(), 3);
    EXPECT_EQ(matxd.cols(), 3);
    // check that the values are ordered properly
    EXPECT_LT(fabs(matxd(0, 2) - (0.2)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 0) - (1.0)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 1) - (1.1)), 1e-6);
    EXPECT_LT(fabs(matxd(2, 0) - (2.0)), 1e-6);

    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixDynamic("test_param_matrix_nonexistent", matxd, Eigen::MatrixXd::Zero(1, 2), 1, 2));
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixDynamic2<double>("test_param_matrix_nonexistent", 16 * Eigen::MatrixXd::Ones(5, 5), 5, 5);
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixDynamic2("test_param_matrix_nonexistent", 5, 6);
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  despin();

  clock->sleep_for(1s);
}
//}

/* TEST_F(Test, param_loader_load_from_file2) //{ */

TEST_F(Test, param_loader_load_from_file2) {

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"file_name", test_resources_path.string() + "/test_config2.yaml"},
  });

  initialize(options.use_intra_process_comms(false).allow_undeclared_parameters(true));

  auto clock = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "[%s]: initializing ParamLoader", node_->get_name());
  auto pl = mrs_lib::ParamLoader(node_, node_->get_name());

  RCLCPP_INFO(node_->get_logger(), "[%s]: testing ParamLoader", node_->get_name());

  bool test_bool = false;
  EXPECT_FALSE(pl.loadParam("test_bool", test_bool));
  pl.resetLoadedSuccessfully();

  EXPECT_FALSE(pl.addYamlFile(""));
  EXPECT_FALSE(pl.addYamlFile(test_resources_path.string() + "/custom_test_config.yaml"));
  pl.resetLoadedSuccessfully();

  EXPECT_TRUE(pl.addYamlFile(test_resources_path.string() + "/test_config.yaml"));
  EXPECT_TRUE(pl.addYamlFileFromParam("file_name"));

  std::vector<Eigen::MatrixXd> loaded_nd_matrix = pl.loadMatrixArray2("test_namespace/test_param_nd_matrix");
  EXPECT_TRUE(pl.loadedSuccessfully());

  if (pl.loadedSuccessfully()) {
    RCLCPP_INFO(node_->get_logger(), "[%s]: parameter loaded OK", node_->get_name());
    int it = 0;
    for (const auto& mat : loaded_nd_matrix) {
      std::cout << "matrix #" << it << std::endl;
      std::cout << mat << std::endl;
      it++;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: parameter loading failure", node_->get_name());
  }

  std::vector<Eigen::MatrixXd> vec;
  pl.resetUniques();
  pl.loadMatrixArray("test_namespace/test_param_nd_matrix", vec);
  EXPECT_TRUE(pl.loadedSuccessfully());

  pl.resetUniques();
  pl.loadMatrixArray("test_namespace/test_param_nd_matrix", vec, vec);
  EXPECT_TRUE(pl.loadedSuccessfully());

  pl.resetUniques();
  vec = pl.loadMatrixArray2("test_namespace/test_param_nd_matrix");
  EXPECT_TRUE(pl.loadedSuccessfully());

  pl.resetUniques();
  vec = pl.loadMatrixArray2("test_namespace/test_param_nd_matrix", vec);
  EXPECT_TRUE(pl.loadedSuccessfully());

  EXPECT_LT(fabs(loaded_nd_matrix.at(0)(0, 0) - (-5)), 1e-6);
  EXPECT_LT(fabs(loaded_nd_matrix.at(1)(2, 0) - (4)), 1e-6);

  std::cout << "Loading empty matrix" << std::endl;
  pl.resetUniques();
  Eigen::Matrix<double, 0, 0> mat = pl.loadMatrixStatic2<0, 0>("test_param_matrix_empty");
  std::cout << "Empty matrix:" << std::endl << mat;

  std::cout << "Testing different matrix loading" << std::endl;

  // | -------------------- loadMatrixStatic -------------------- |
  // reset the loadedSuccessfully flag back to true
  pl.resetLoadedSuccessfully();

  {
    Eigen::Matrix3d mat3d;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixStatic("test_namespace/test_param_matrix_3x3", mat3d));
    // check that the values are ordered properly
    EXPECT_LT(fabs(mat3d(0, 1) - (0.1)), 1e-6);
    EXPECT_LT(fabs(mat3d(1, 2) - (1.2)), 1e-6);
    EXPECT_LT(fabs(mat3d(2, 0) - (2.0)), 1e-6);
    EXPECT_LT(fabs(mat3d(2, 2) - (2.2)), 1e-6);

    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixStatic("test_namespace/test_param_matrix_3x3", mat3d, Eigen::Matrix3d::Identity()));
    pl.resetUniques();
    mat3d = pl.loadMatrixStatic2<3, 3, double>("test_namespace/test_param_matrix_3x3");
    pl.resetUniques();
    mat3d = pl.loadMatrixStatic2<3, 3, double>("test_namespace/test_param_matrix_nonexistent", Eigen::Matrix3d::Identity() * Eigen::Matrix3d::Ones());
    EXPECT_TRUE(pl.loadedSuccessfully());

    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixStatic("test_namespace/test_param_matrix_nonexistent", mat3d, Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(pl.loadedSuccessfully());

    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixStatic("test_namespace/test_param_matrix_empty", mat3d));
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  // | --------------------- loadMatrixKnown -------------------- |
  // reset the loadedSuccessfully flag back to true
  pl.resetLoadedSuccessfully();

  {
    Eigen::MatrixXd matxd;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixKnown("test_namespace/test_param_matrix_4x2", matxd, 4, 2));
    EXPECT_TRUE(pl.loadedSuccessfully());
    // check that the values are ordered properly
    EXPECT_LT(fabs(matxd(0, 1) - (0.1)), 1e-6);
    EXPECT_LT(fabs(matxd(2, 0) - (2.0)), 1e-6);
    EXPECT_LT(fabs(matxd(3, 1) - (3.1)), 1e-6);

    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixKnown("test_namespace/test_param_matrix_nonexistent", matxd, Eigen::MatrixXd::Identity(15, 16), 15, 16));
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixKnown2<double>("test_namespace/test_param_matrix_nonexistent", Eigen::MatrixXd::Identity(15, 16), 15, 16);
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixKnown2<double>("test_namespace/test_param_matrix_nonexistent", 15, 16);
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  // | -------------------- loadMatrixDynamic ------------------- |
  // reset the loadedSuccessfully flag back to true
  pl.resetLoadedSuccessfully();

  {
    Eigen::MatrixXd matxd;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixDynamic("test_namespace/test_param_matrix_3x3", matxd, 3, -1));
    EXPECT_TRUE(pl.loadedSuccessfully());
    EXPECT_EQ(matxd.rows(), 3);
    EXPECT_EQ(matxd.cols(), 3);
    // check that the values are ordered properly
    EXPECT_LT(fabs(matxd(0, 2) - (0.2)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 0) - (1.0)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 1) - (1.1)), 1e-6);
    EXPECT_LT(fabs(matxd(2, 0) - (2.0)), 1e-6);

    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixDynamic("test_namespace/test_param_matrix_3x3", matxd, -1, 3));
    EXPECT_TRUE(pl.loadedSuccessfully());
    EXPECT_EQ(matxd.rows(), 3);
    EXPECT_EQ(matxd.cols(), 3);
    // check that the values are ordered properly
    EXPECT_LT(fabs(matxd(0, 2) - (0.2)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 0) - (1.0)), 1e-6);
    EXPECT_LT(fabs(matxd(1, 1) - (1.1)), 1e-6);
    EXPECT_LT(fabs(matxd(2, 0) - (2.0)), 1e-6);

    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixDynamic("test_namespace/test_param_matrix_nonexistent", matxd, Eigen::MatrixXd::Zero(1, 2), 1, 2));
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixDynamic2<double>("test_namespace/test_param_matrix_nonexistent", 16 * Eigen::MatrixXd::Ones(5, 5), 5, 5);
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixDynamic2("test_namespace/test_param_matrix_nonexistent", 5, 6);
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  despin();

  clock->sleep_for(1s);
}
//}
