// clang: MatousFormat
/**  \file
     \brief Example file for the ParamLoader convenience class.
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib param_loader_example`.

     See \ref param_loader/example.cpp.
 */

/**  \example "param_loader/example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib param_loader_example`.
     It demonstrates loading of parameters from the `rosparam` server inside your node/nodelet using `ParamLoader`.

     To load parameters into the `rosparam` server, use a launchfile prefferably.
     See documentation of ROS launchfiles here: http://wiki.ros.org/roslaunch/XML.
     Specifically, the `param` XML tag is used for loading parameters directly from the launchfile: http://wiki.ros.org/roslaunch/XML/param,
     and the `rosparam` XML tag tag is used for loading parameters from a `yaml` file: http://wiki.ros.org/roslaunch/XML/rosparam.

     Example code for using the \ref ParamLoader (see documentation of the ParamLoader class):
 */

/* Include the ParamLoader header */
#include <mrs_lib/param_loader.h>

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
  pl.loadParam("test_param_double", test_param_double);

  /* Load a parameter and return it */ 
  const std::vector<int> test_param_vector = pl.loadParam2<std::vector<int>>("test_param_vector");

  /* Load a parameter with a default value, which will be used in this case
   * unless you manually push the parameter 'test_param_int' to the rosparam server
   * (e.g. using 'rosparam set test_param_int 15'). */ 
  [[maybe_unused]] const int test_param_int = pl.loadParam2<int>("test_param_int", 4);

  /* Load a compulsory parameter - without a default value. This will fail in this
   * case, unless you manually push the parameter to the server. */ 
  [[maybe_unused]] const bool test_param_bool = pl.loadParam2<bool>("test_param_bool");

  /* Check if all parameters were loaded successfully */ 
  if (!pl.loadedSuccessfully())
  {
    /* If not, alert the user and shut the node down */ 
    ROS_ERROR("[%s]: parameter loading failure", node_name.c_str());
    ros::shutdown();
    return 1;
  }

  /* Otherwise, continue the program (in this case it's the end) */ 
  return 0;

}

