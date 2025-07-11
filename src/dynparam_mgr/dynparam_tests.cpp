#include <mrs_lib/param_provider.h>
#include <mrs_lib/dynparam_mgr.h>

template <typename T>
void callback(const std::string& param_name, const T& new_value)
{
  std::cout << "parameter \"" << param_name << "\" changed: " << new_value << std::endl;
}

template <typename T>
using range_t = mrs_lib::DynparamMgr::range_t<T>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dynparam_tests");

  std::mutex mtx;
  auto dynparam_mgr = mrs_lib::DynparamMgr(node, mtx);
  auto& param_provider = dynparam_mgr.get_param_provider();

  param_provider.addYamlFile("/home/matous/workspace/src/mrs_uav_core/ros_packages/mrs_lib/test/param_loader/test_config.yaml");

  int test_int;

  const mrs_lib::DynparamMgr::update_cbk_t<int> cbk = std::bind(&callback<int>, "test_int", std::placeholders::_1);
  const auto result = dynparam_mgr.register_param("test_int", &test_int, {-1, 100}, cbk);
  std::cout << "register param \"test_int\": " << result << std::endl;

  const auto resolved_name = param_provider.resolveName("test_int");
  std::cout << "resolve name \"test_int\": " << resolved_name << std::endl;

  const auto result2 = param_provider.getParam("test_int", test_int);
  std::cout << "get param \"test_int\": " << result2 << std::endl;

  const auto result3 = param_provider.getParam("param_provider/test_int", test_int);
  std::cout << "get param \"param_provider/test_int\": " << result3 << std::endl;

  dynparam_mgr.register_param("test_int", &test_int, 15, {-1, 100});
  dynparam_mgr.register_param("test_int", &test_int, 15);

  rclcpp::spin(node);

  return 0;
}
