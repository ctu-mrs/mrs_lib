#include <mrs_lib/param_loader.h>

// Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
// Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

template bool mrs_lib::ParamLoader::load_param<bool>(const std::string& name, bool& out_value, const bool& default_value, bool dynamic);
template bool mrs_lib::ParamLoader::load_param2<bool>(const std::string& name, const bool& default_value);

template bool mrs_lib::ParamLoader::load_param<int>(const std::string& name, int& out_value, const int& default_value, bool dynamic);
template int mrs_lib::ParamLoader::load_param2<int>(const std::string& name, const int& default_value);

template bool mrs_lib::ParamLoader::load_param<double>(const std::string& name, double& out_value, const double& default_value, bool dynamic);
template double mrs_lib::ParamLoader::load_param2<double>(const std::string& name, const double& default_value);

template bool mrs_lib::ParamLoader::load_param<std::string>(const std::string& name, std::string& out_value, const std::string& default_value, bool dynamic);
template std::string mrs_lib::ParamLoader::load_param2<std::string>(const std::string& name, const std::string& default_value);

