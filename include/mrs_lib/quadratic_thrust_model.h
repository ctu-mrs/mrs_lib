#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <cmath>

namespace mrs_lib
{

namespace quadratic_thrust_model
{

typedef struct
{
  double A;
  double B;
  int    n_motors;
} MotorParams_t;

double inline thrustToForce(const MotorParams_t motor_params, const double thrust) {

  return motor_params.n_motors * pow((thrust - motor_params.B) / motor_params.A, 2);
}

double inline forceToThrust(const MotorParams_t motor_params, const double force) {

  return sqrt(force / motor_params.n_motors) * motor_params.A + motor_params.B;
}

}  // namespace quadratic_thrust_model

}  // namespace mrs_lib

#endif  // QUADRATIC_THRUST_MODEL_H
