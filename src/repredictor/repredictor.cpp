// clang: MatousFormat

#include <mrs_lib/repredictor.h>
#include <mrs_lib/lkf.h>

namespace mrs_lib
{
  constexpr int mrsrep_n_states = 3;
  constexpr int mrsrep_n_inputs = 3;
  constexpr int mrsrep_n_measurements = Eigen::Dynamic;
  template class Repredictor<mrsrep_n_states, mrsrep_n_inputs, mrsrep_n_measurements, LKF_MRS_odom>;
  using MRSRepredictor = Repredictor<mrsrep_n_states, mrsrep_n_inputs, mrsrep_n_measurements, LKF_MRS_odom>;
}  // namespace mrs_lib
