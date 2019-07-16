/* author: Tomas Baca */

#include <ros/ros.h>
#include <mrs_lib/ukf.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

namespace mrs_lib
{
  const int n_states = 3;
  const int n_inputs = 3;
  const int n_measurements = 3;
  UKF<n_states, n_inputs, n_measurements> ukf;
}
