// clang: MatousFormat
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/internal/version_macros.hpp>

namespace mrs_lib::MRS_LIB_INTERNAL_INLINE_API_V1 v1
{
  namespace geometry
  {

    // to ensure these classes are generated
    template struct cyclic<double, radians>;
    template struct cyclic<double, sradians>;
    template struct cyclic<double, degrees>;
    template struct cyclic<double, sdegrees>;

  } // namespace geometry
} // namespace mrs_lib::inline v1
