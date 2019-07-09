#include <mrs_lib/ParamLoader.h>

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("param_loader_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  ROS_INFO("[%s]: pushing testing parameters to the rosparam server", node_name.c_str());
  nh.setParam("test_param_double", std::numeric_limits<double>::quiet_NaN());
  nh.setParam("test_param_vector", std::vector<int>({6, 6, 6}));

  ROS_INFO("[%s]: loading parameters using ParamLoader", node_name.c_str());
  mrs_lib::ParamLoader pl(nh, node_name);
  
  /* Most basic way to load a parameter to a variable, which could be a class member */ 
  double test_param_double;
  pl.load_param("test_param_double", test_param_double);

  /* Load a parameter and return it */ 
  const std::vector<int> test_param_vector = pl.load_param2<std::vector<int>>("test_param_vector");

  /* Load a parameter with a default value, which will be used in this case
   * unless you manually push the parameter 'test_param_int' to the rosparam server
   * (e.g. using 'rosparam set test_param_int 15'). */ 
  const int test_param_int = pl.load_param2<int>("test_param_int", 4);

  /* Load a compulsory parameter - without a default value. This will fail in this
   * case, unless you manually push the parameter to the server. */ 
  const bool test_param_bool = pl.load_param2<bool>("test_param_bool");

  /* Check if all parameters were loaded successfully */ 
  if (!pl.loaded_successfully())
  {
    /* If not, alert the user and shut the node down */ 
    ROS_ERROR("[%s]: parameter loading failure", node_name.c_str());
    ros::shutdown();
    return 1;
  }

  /* Otherwise, continue the program (in this case it's the end) */ 
  return 0;

}

