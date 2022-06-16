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
  pl.loadParam("services", list, XmlRpc::XmlRpcValue());

  ROS_INFO("[%s]: testing ParamLoader", ros::this_node::getName().c_str());
  std::vector<Eigen::MatrixXd> loaded_nd_matrix = pl.loadMatrixArray2("test_param_nd_matrix");
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
    result *= 0;
  }

  std::cout << "Loading empty matrix" << std::endl;
  Eigen::Matrix<double, 0, 0> mat = pl.loadMatrixStatic2<0, 0>("test_param_matrix");
  std::cout << "Empty matrix:" << std::endl << mat;

  if (fabs(loaded_nd_matrix.at(0)(0, 0) - (-5)) > 1e-6) {
    ROS_INFO("[%s]: (0, 0) of the fist matrix failed", ros::this_node::getName().c_str());
    result *= 0;
  }

  if (fabs(loaded_nd_matrix.at(1)(2, 0) - (4)) > 1e-6) {
    ROS_INFO("[%s]: (2, 0) of the second matrix failed", ros::this_node::getName().c_str());
    result *= 0;
  }

  // these are to check that the code compiles, it doesn't check anything during runtime
  std::cout << "Testing different matrix loading" << std::endl;

  Eigen::Matrix3f mat3f;
  pl.loadMatrixStatic("test_param_matrix", mat3f);
  pl.loadMatrixStatic("test_param_matrix", mat3f, Eigen::Matrix3f::Identity());
  mat3f = pl.loadMatrixStatic2<3, 3, float>("test_param_matrix");
  mat3f = pl.loadMatrixStatic2<3, 3, float>("test_param_matrix", Eigen::Matrix3f::Identity()*Eigen::Matrix3f::Ones());

  Eigen::MatrixXf matxf;
  pl.loadMatrixStatic("test_param_matrix", matxf, 15, 16);
  pl.loadMatrixStatic("test_param_matrix", matxf, Eigen::MatrixXf::Identity(15, 16), 15, 16);
  matxf = pl.loadMatrixStatic2<float>("test_param_matrix", 15, 16);
  matxf = pl.loadMatrixStatic2<float>("test_param_matrix", Eigen::MatrixXf::Identity(15, 16), 15, 16);

  Eigen::MatrixXd matxd;
  pl.loadMatrixDynamic("test_param_matrix", matxd, 1, 2);
  pl.loadMatrixDynamic("test_param_matrix", matxd, Eigen::MatrixXd::Zero(1, 2), 1, 2);
  matxd = pl.loadMatrixDynamic2("test_param_matrix", 5, 6);
  matxd = pl.loadMatrixDynamic2<double>("test_param_matrix", 16*Eigen::MatrixXd::Ones(5, 5), 5, 5);

  std::vector<Eigen::MatrixXd> vec;
  pl.loadMatrixArray("test_param_matrix", vec);
  pl.loadMatrixArray("test_param_matrix", vec, vec);
  vec = pl.loadMatrixArray2("test_param_matrix");
  vec = pl.loadMatrixArray2("test_param_matrix", vec);

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "param_loader_tests");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
