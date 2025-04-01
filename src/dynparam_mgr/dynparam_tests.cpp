#include <mrs_lib/param_provider.h>
#include <mrs_lib/dynamic_params.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dynparam_tests");

  std::mutex mtx;
  auto dynparam_mgr = mrs_lib::DynparamMgr(node, mtx, node->get_name());

  bool test_bool;

  const auto result = dynparam_mgr.register_param("test_bool", test_bool);
  std::cout << "register param \"test_bool\": " << result << std::endl;

  auto param_provider = mrs_lib::ParamProvider(node, node->get_name(), true);
  param_provider.addYamlFile("/home/matous/workspace/src/mrs_lib/test/param_loader/test_config.yaml");

  const auto result2 = param_provider.getParam("test_bool", test_bool);
  std::cout << "get param \"test_bool\": " << result2 << std::endl;

  const auto result3 = param_provider.getParam("param_provider/test_bool", test_bool);
  std::cout << "get param \"param_provider/test_bool\": " << result3 << std::endl;

  return 0;
}
