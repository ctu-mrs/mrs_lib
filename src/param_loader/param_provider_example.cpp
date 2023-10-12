#include <mrs_lib/param_provider.h>

template<typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
    out << "{";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "}";
    return out;
}

template <typename T>
void test_load_param(const std::string& param_name, const mrs_lib::ParamProvider pp)
{
  T param;
  const bool success = pp.getParam(param_name, param);
  if (success)
    std::cout << "loading of parameter \"" << param_name << "\" was successful: " << param << std::endl;
  else
    std::cout << "loading of parameter \"" << param_name << "\" was NOT successful!" << std::endl;
}

int main(int argc, char **argv)
{
  const std::string node_name("param_provider_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");
  mrs_lib::ParamProvider pp(nh, "ParamProviderExample");

  pp.addYamlFile("/tmp/test.yaml");

  test_load_param<double>("test/namespace/a", pp);
  test_load_param<double>("test/b", pp);
  test_load_param<std::string>("c", pp);
  test_load_param<std::vector<std::string>>("text_array", pp);

  return 0;
}
