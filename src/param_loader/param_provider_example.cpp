#include <mrs_lib/param_provider.h>


int main(int argc, char **argv)
{
  const std::string node_name("param_provider_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  mrs_lib::ParamProvider pp(nh);

  pp.addYamlFile("/tmp/test.yaml");
  double param;
  std::cout << "loading was success: " << pp.getParam("test/a", param) << std::endl;
  std::cout << "loaded value: " << param << std::endl;

  std::cout << "loading was success: " << pp.getParam("test/b", param) << std::endl;
  std::cout << "loaded value: " << param << std::endl;

  return 0;
}
