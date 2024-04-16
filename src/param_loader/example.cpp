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
  ros::NodeHandle nh("~");

  /* Set up some parameters to be loaded by the example. In normal usage,
   * you don't have to do this. Parameters are loaded from the rosparam
   * server or directly from a YAML file. */
  ROS_INFO_STREAM("[" << node_name << "]: pushing testing parameters to the rosparam server");
  nh.setParam("test_param_double", std::numeric_limits<double>::quiet_NaN());
  nh.setParam("test_param_vector", std::vector<int>({6, 6, 6}));
  /* nh.setParam("test_param_matrix_3x3", std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7, 8})); */

  /* Initialize the param loader with a NodeHandle and optionally the name of this node. */
  ROS_INFO_STREAM("[" << node_name <<" ]: loading parameters using ParamLoader");
  mrs_lib::ParamLoader pl(nh, node_name);
  
  /* Most basic way to load a parameter to a variable, which could be a class member */ 
  double test_param_double;
  pl.loadParam("test_param_double", test_param_double);

  /* Load a parameter and return it */ 
  const auto test_param_vector = pl.loadParam2<std::vector<int>>("test_param_vector");

  /* Load a parameter with a default value, which will be used in this case
   * unless you manually push the parameter 'test_param_int' to the rosparam server
   * (e.g. using 'rosparam set test_param_int 15'). */ 
  const auto test_param_int = pl.loadParam2<int>("test_param_int", 4);

  /* Load a compulsory parameter - without a default value. This will fail in this
   * case, unless you manually push the parameter to the server. */ 
  const auto test_param_bool = pl.loadParam2<bool>("test_param_bool");

  Eigen::MatrixXd matxd;
  pl.loadMatrixDynamic("test_param_matrix_3x3", matxd, -1, 3);
  ROS_INFO_STREAM("[" << node_name << "]: Loaded matrix: " << matxd);

  /* So far, all the parameters were loaded from the ROS parameter server.
   * These must be loaded to the server using the param or rosparam commands
   * in the launchfile (or manually using the CLI tools). However, loading
   * a large number of parameters from the ROS server in parallel can be slow,
   * so we also offer the possibility to load parameters directly from a YAML file.
   * To do this, firstly you tell the ParamLoader which file to use: */
  pl.addYamlFile("/tmp/test.yaml");
  /* (Note that this file will have to be created manually in this case for this */
  /*  example to work.) */

  /* Then, you can load parameters specified in the file normally. */
  const auto string_from_yaml = pl.loadParam2<std::string>("string_from_yaml");

  /* Check if all parameters were loaded successfully */ 
  if (!pl.loadedSuccessfully())
  {
    /* If not, alert the user and shut the node down */ 
    ROS_ERROR_STREAM("[" << node_name << "]: parameter loading failure! Ending the node");
    ros::shutdown();
    return 1;
  }

  /* Otherwise, continue the program and happily use the loaded parameters! */ 
  /* This is just some nonsense usage for demonstration. */
  if (test_param_bool)
  {
    ROS_INFO_STREAM("[" << node_name << "]: Showing values of: " << string_from_yaml);
    for (auto& el : test_param_vector)
      ROS_INFO_STREAM("[" << node_name << "]: Value: " << test_param_double * el + test_param_int);
  }

  return 0;

}

