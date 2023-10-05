#include <string>

namespace mrs_lib
{
  class RosParamProvider : public ParamProvider
  {
    public:

      template <typename T>
      bool getParam(const std::string& param_name, T& value_out);

      ParamProvider() = delete;
  };
}
