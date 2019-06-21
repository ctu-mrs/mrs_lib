#include <mrs_lib/ParamLoader.h>

std::vector<double> gemerate_nd_matrix(int rows, int cols, int dims)
{
  std::vector<double> ret;
  ret.reserve(rows*cols*dims);
  for (int it = 0; it < rows*cols*dims; it++)
  {
    ret.push_back(it);
  }
  return ret;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "param_loader_tests");
  ros::NodeHandle nh;

  int rows = 4;
  int cols = 3;
  int dims = 3;

  ROS_INFO("[%s]: pushing testing parameters to the rosparam server", ros::this_node::getName().c_str());
  std::vector<double> param_nd_matrix = gemerate_nd_matrix(rows, cols, dims);
  nh.setParam("test_param_nd_matrix", param_nd_matrix);

  ROS_INFO("[%s]: initializing ParamLoader", ros::this_node::getName().c_str());
  mrs_lib::ParamLoader pl(nh);

  ROS_INFO("[%s]: testing ParamLoader", ros::this_node::getName().c_str());
  std::vector<Eigen::MatrixXd> loaded_nd_matrix = pl.load_matrix_array2("test_param_nd_matrix", rows, cols, dims);
  int it = 0;
  for (const auto& mat : loaded_nd_matrix)
  {
    std::cout << "matrix #" << it << std::endl;
    std::cout << mat << std::endl;
    it++;
  }

  return 0;
}
