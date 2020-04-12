#include <mrs_lib/utils.h>

namespace mrs_lib
{

ScopeUnset::ScopeUnset(bool &in) : variable(in) {

  variable = true;
}

ScopeUnset::~ScopeUnset() {

  variable = false;
}

}  // namespace mrs_lib
