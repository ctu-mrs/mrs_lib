#include <mrs_lib/ParamLoader.h>

// Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
// Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

template bool mrs_lib::ParamLoader::load_param<bool>(const std::string& name, bool& out_value, const bool& default_value);
template bool mrs_lib::ParamLoader::load_param_reusable<bool>(const std::string& name, const bool& default_value);

template bool mrs_lib::ParamLoader::load_param<int>(const std::string& name, int& out_value, const int& default_value);
template int mrs_lib::ParamLoader::load_param_reusable<int>(const std::string& name, const int& default_value);

template bool mrs_lib::ParamLoader::load_param<double>(const std::string& name, double& out_value, const double& default_value);
template double mrs_lib::ParamLoader::load_param_reusable<double>(const std::string& name, const double& default_value);

template bool mrs_lib::ParamLoader::load_param<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);
template std::string mrs_lib::ParamLoader::load_param_reusable<std::string>(const std::string& name, const std::string& default_value);
