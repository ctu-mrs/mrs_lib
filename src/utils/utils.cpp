#include <mrs_lib/utils.h>

namespace mrs_lib
{

AtomicScopeFlag::AtomicScopeFlag(std::atomic<bool>& in)
  : variable(in)
{
  variable = true;
}

AtomicScopeFlag::~AtomicScopeFlag()
{
  variable = false;
}

}  // namespace mrs_lib
