#include <mrs_lib/utils.h>
#include <mrs_lib/internal/version_macros.hpp>

namespace mrs_lib::MRS_LIB_INTERNAL_INLINE_API_V1 v1
{

  AtomicScopeFlag::AtomicScopeFlag(std::atomic<bool>& in) : variable(in)
  {
    variable = true;
  }

  AtomicScopeFlag::~AtomicScopeFlag()
  {
    variable = false;
  }

} // namespace mrs_lib::inline v1
