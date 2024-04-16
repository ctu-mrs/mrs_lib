#include <mrs_lib/param_loader.h>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace mrs_lib;
using namespace std;

std::vector<double> gemerate_nd_matrix(int rows, const std::vector<int>& cols)
{
  std::vector<double> ret;
  int tot_cols = 0;
  for (const auto& col : cols)
    tot_cols += col;
  ret.reserve(rows*tot_cols);
  for (int it = 0; it < rows*tot_cols; it++)
  {
    ret.push_back(it);
  }
  return ret;
}

/* TEST(TESTSuite, main_test) //{ */

TEST(TESTSuite, main_test) {

  int result = 1;

  // Set up ROS.
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing ParamLoader", ros::this_node::getName().c_str());
  mrs_lib::ParamLoader pl(nh);

  XmlRpc::XmlRpcValue list;
  EXPECT_TRUE(pl.loadParam("services", list, XmlRpc::XmlRpcValue()));

  ROS_INFO("[%s]: testing ParamLoader", ros::this_node::getName().c_str());
  std::vector<Eigen::MatrixXd> loaded_nd_matrix = pl.loadMatrixArray2("test_param_nd_matrix");
  EXPECT_TRUE(pl.loadedSuccessfully());

  if (pl.loadedSuccessfully())
  {
    ROS_INFO("[%s]: parameter loaded OK", ros::this_node::getName().c_str());
    int it = 0;
    for (const auto& mat : loaded_nd_matrix)
    {
      std::cout << "matrix #" << it << std::endl;
      std::cout << mat << std::endl;
      it++;
    }
  } else
  {
    ROS_ERROR("[%s]: parameter loading failure", ros::this_node::getName().c_str());
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
    Eigen::Matrix3f mat3f;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixStatic("test_param_matrix_3x3", mat3f));
    // check that the values are ordered properly
    EXPECT_LT(fabs(mat3f(0, 1) - (0.1)), 1e-6);
    EXPECT_LT(fabs(mat3f(1, 2) - (1.2)), 1e-6);
    EXPECT_LT(fabs(mat3f(2, 0) - (2.0)), 1e-6);
    EXPECT_LT(fabs(mat3f(2, 2) - (2.2)), 1e-6);
    
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixStatic("test_param_matrix_3x3", mat3f, Eigen::Matrix3f::Identity()));
    pl.resetUniques();
    mat3f = pl.loadMatrixStatic2<3, 3, float>("test_param_matrix_3x3");
    pl.resetUniques();
    mat3f = pl.loadMatrixStatic2<3, 3, float>("test_param_matrix_nonexistent", Eigen::Matrix3f::Identity()*Eigen::Matrix3f::Ones());
    EXPECT_TRUE(pl.loadedSuccessfully());
    
    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixStatic("test_param_matrix_nonexistent", mat3f, Eigen::Matrix3f::Identity()));
    EXPECT_TRUE(pl.loadedSuccessfully());
    
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixStatic("test_param_matrix_empty", mat3f));
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  // | --------------------- loadMatrixKnown -------------------- |
  // reset the loadedSuccessfully flag back to true
  pl.resetLoadedSuccessfully();

  {
    Eigen::MatrixXf matxf;
    pl.resetUniques();
    EXPECT_TRUE(pl.loadMatrixKnown("test_param_matrix_4x2", matxf, 4, 2));
    EXPECT_TRUE(pl.loadedSuccessfully());
    // check that the values are ordered properly
    EXPECT_LT(fabs(matxf(0, 1) - (0.1)), 1e-6);
    EXPECT_LT(fabs(matxf(2, 0) - (2.0)), 1e-6);
    EXPECT_LT(fabs(matxf(3, 1) - (3.1)), 1e-6);
    
    // with optional parameters, it should return false if failed to load, but not set the loadedSuccessfully flag
    pl.resetUniques();
    EXPECT_FALSE(pl.loadMatrixKnown("test_param_matrix_nonexistent", matxf, Eigen::MatrixXf::Identity(15, 16), 15, 16));
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxf = pl.loadMatrixKnown2<float>("test_param_matrix_nonexistent", Eigen::MatrixXf::Identity(15, 16), 15, 16);
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxf = pl.loadMatrixKnown2<float>("test_param_matrix_nonexistent", 15, 16);
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
    matxd = pl.loadMatrixDynamic2<double>("test_param_matrix_nonexistent", 16*Eigen::MatrixXd::Ones(5, 5), 5, 5);
    EXPECT_TRUE(pl.loadedSuccessfully());
    pl.resetUniques();
    matxd = pl.loadMatrixDynamic2("test_param_matrix_nonexistent", 5, 6);
    EXPECT_FALSE(pl.loadedSuccessfully());
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, static_params_test) //{ */

TEST(TESTSuite, static_params_test) {

  // Set up ROS.
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing ParamLoader", ros::this_node::getName().c_str());
  mrs_lib::ParamLoader pl(nh);
  EXPECT_TRUE(pl.addYamlFileFromParam("static_params_file"));
  std::string static_params_file;
  EXPECT_TRUE(pl.loadParamReusable("static_params_file", static_params_file));
  EXPECT_TRUE(pl.addYamlFile(static_params_file));

  ROS_INFO("[%s]: testing ParamLoader", ros::this_node::getName().c_str());

  std::vector<std::string> static_param;
  EXPECT_TRUE(pl.loadParam("static_param", static_param));
  EXPECT_TRUE(pl.loadedSuccessfully());

  double static_param2;
  EXPECT_TRUE(pl.loadParam("ns1/ns2/a", static_param2));
  EXPECT_TRUE(pl.loadedSuccessfully());

  EXPECT_FALSE(pl.loadParam("static_param_asdf", static_param));
  EXPECT_FALSE(pl.loadedSuccessfully());
}

//}

/* TEST(TESTSuite, weird_types_test) //{ */

TEST(TESTSuite, weird_types_test) {

  // Set up ROS.
  ros::NodeHandle nh = ros::NodeHandle("~");

  ros::Time::waitForValid();

  ROS_INFO("[%s]: initializing ParamLoader", ros::this_node::getName().c_str());
  mrs_lib::ParamLoader pl(nh);
  XmlRpc::XmlRpcValue default_val;
  XmlRpc::XmlRpcValue xml_val;
  EXPECT_FALSE(pl.loadParam("XmlRpc_test", xml_val, default_val));
  EXPECT_TRUE(pl.loadedSuccessfully());
  EXPECT_FALSE(pl.loadParam("XmlRpc_test", xml_val));
  EXPECT_FALSE(pl.loadedSuccessfully());
  [[maybe_unused]] const auto ret = pl.loadParam2("XmlRpc_test", default_val);

  EXPECT_TRUE(pl.loadParam("rosparam_xmlrpc_value", xml_val));
  EXPECT_TRUE(pl.addYamlFileFromParam("static_params_file"));
  EXPECT_FALSE(pl.loadParam("static_xmlrpc_value", xml_val));

  EXPECT_FALSE(pl.loadedSuccessfully());
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "param_loader_tests");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
