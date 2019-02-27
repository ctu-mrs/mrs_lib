#include "mrs_lib/Repredictor.h"

namespace mrs_lib
{
  constexpr int mrsrep_n_states = 6;
  constexpr int mrsrep_n_inputs = 6;
  constexpr int mrsrep_n_measurements = Eigen::Dynamic;
  template
  class Repredictor<mrsrep_n_states, mrsrep_n_inputs, mrsrep_n_measurements>;
  using MRSRepredictor = Repredictor<mrsrep_n_states, mrsrep_n_inputs, mrsrep_n_measurements>;
}
