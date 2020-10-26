// clang: MatousFormat
#include <mrs_lib/geometry_utils.h>

using quat_t = Eigen::Quaterniond;

namespace mrs_lib
{
  namespace geometry
  {

    // instantiation of common template values
    vec_t<3 + 1> to_homogenous(const vec_t<3>& vec);
    vec_t<2 + 1> to_homogenous(const vec_t<2>& vec);

  }  // namespace geometry
}  // namespace mrs_lib
