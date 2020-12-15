// clang: MatousFormat

#include <mrs_lib/repredictor.h>
#include <mrs_lib/lkf.h>

namespace mrs_lib
{
  template class Repredictor<LKF_MRS_odom>;
  using MRSRepredictor = Repredictor<LKF_MRS_odom>;
}  // namespace mrs_lib
