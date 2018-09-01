#include <mrs_lib/Utils.h>

namespace mrs_lib
{

  ContextUnset::ContextUnset(bool &in) : variable(in) {
  
    variable = true;
  }

  ContextUnset::~ContextUnset() {

    variable = false;
  }

}  // namespace mrs_lib
