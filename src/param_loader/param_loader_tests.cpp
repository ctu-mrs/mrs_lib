#include <mrs_lib/ParamLoader.h>

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

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "param_loader_tests");
  ros::NodeHandle nh = ros::NodeHandle("~");

  /* int rows = 4; */
  /* std::vector<int> cols = {3, 2, 1}; */

  /* ROS_INFO("[%s]: pushing testing parameters to the rosparam server", ros::this_node::getName().c_str()); */
  /* std::vector<double> param_nd_matrix = gemerate_nd_matrix(rows, cols); */
  /* nh.setParam("test_param_nd_matrix/rows", rows); */
  /* nh.setParam("test_param_nd_matrix/cols", cols); */
  /* nh.setParam("test_param_nd_matrix/data", param_nd_matrix); */

  ROS_INFO("[%s]: initializing ParamLoader", ros::this_node::getName().c_str());
  mrs_lib::ParamLoader pl(nh);

  XmlRpc::XmlRpcValue list;
  pl.load_param("services", list, XmlRpc::XmlRpcValue());

  ROS_INFO("[%s]: testing ParamLoader", ros::this_node::getName().c_str());
  std::vector<Eigen::MatrixXd> loaded_nd_matrix = pl.load_matrix_array2("test_param_nd_matrix");
  if (pl.loaded_successfully())
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
    return 1;
  }

  std::cout << "Loading empty matrix" << std::endl;
  nh.setParam("test_param_matrix", std::vector<double>());
  Eigen::Matrix<double, 0, 0> mat = pl.load_matrix_static2<0, 0>("test_param_matrix");
  std::cout << "Empty matrix:" << std::endl << mat;

  return 0;
}
