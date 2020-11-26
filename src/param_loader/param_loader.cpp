#include <mrs_lib/param_loader.h>

// Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
// Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

template bool mrs_lib::ParamLoader::loadParam<bool>(const std::string& name, bool& out_value, const bool& default_value);
template bool mrs_lib::ParamLoader::loadParamReusable<bool>(const std::string& name, const bool& default_value);

template bool mrs_lib::ParamLoader::loadParam<int>(const std::string& name, int& out_value, const int& default_value);
template int mrs_lib::ParamLoader::loadParamReusable<int>(const std::string& name, const int& default_value);

template bool mrs_lib::ParamLoader::loadParam<double>(const std::string& name, double& out_value, const double& default_value);
template double mrs_lib::ParamLoader::loadParamReusable<double>(const std::string& name, const double& default_value);

template bool mrs_lib::ParamLoader::loadParam<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);
template std::string mrs_lib::ParamLoader::loadParamReusable<std::string>(const std::string& name, const std::string& default_value);


template <>
ros::Duration mrs_lib::ParamLoader::loadParam2<ros::Duration>(const std::string& name, const ros::Duration& default_value)
{
  const double secs = loadParam2<double>(name, default_value.toSec());
  const ros::Duration ret(secs);
  return ret;
}

template <>
ros::Duration mrs_lib::ParamLoader::loadParam2<ros::Duration>(const std::string& name)
{
  const double secs = loadParam2<double>(name);
  const ros::Duration ret(secs);
  return ret;
}
