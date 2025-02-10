#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <cmath>

namespace mrs_lib
{

namespace quadratic_throttle_model
{

typedef struct
{
  double A;
  double B;
  int    n_motors;
} MotorParams_t;

double inline throttleToForce(const MotorParams_t motor_params, const double throttle) {

  return motor_params.n_motors * pow((throttle - motor_params.B) / motor_params.A, 2);
}

double inline forceToThrottle(const MotorParams_t motor_params, const double force) {

  return sqrt(force / motor_params.n_motors) * motor_params.A + motor_params.B;
}

}  // namespace quadratic_throttle_model

}  // namespace mrs_lib

#endif  // QUADRATIC_THRUST_MODEL_H
