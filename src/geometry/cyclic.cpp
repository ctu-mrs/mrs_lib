// clang: MatousFormat
#include <mrs_lib/geometry/cyclic.h>

namespace mrs_lib
{
  namespace geometry
  {

    // to ensure these classes are generated
    template struct cyclic<double, radians>;
    template struct cyclic<double, sradians>;
    template struct cyclic<double, degrees>;
    template struct cyclic<double, sdegrees>;

  }  // namespace geometry
}  // namespace mrs_lib
