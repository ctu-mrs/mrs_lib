#include <mrs_lib/param_loader.h>

// Explicit instantiation of the tepmplated functions to precompile them into mrs_lib and speed up compilation of user program.
// Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

template bool mrs_lib::ParamLoader::load_param2<bool>(const std::string& name, const bool& default_value);

template int mrs_lib::ParamLoader::load_param2<int>(const std::string& name, const int& default_value);

template double mrs_lib::ParamLoader::load_param2<double>(const std::string& name, const double& default_value);

template std::string mrs_lib::ParamLoader::load_param2<std::string>(const std::string& name, const std::string& default_value);

