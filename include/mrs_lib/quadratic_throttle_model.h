#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <cmath>
#include <mrs_lib/internal/version_macros.hpp>

namespace mrs_lib::MRS_LIB_INTERNAL_INLINE_API_V1 v1
{

  namespace quadratic_throttle_model
  {

    typedef struct
    {
      double A;
      double B;
      int n_motors;
    } MotorParams_t;

    double inline throttleToForce(const MotorParams_t motor_params, const double throttle)
    {

      return motor_params.n_motors * pow((throttle - motor_params.B) / motor_params.A, 2);
    }

    double inline forceToThrottle(const MotorParams_t motor_params, const double force)
    {

      return sqrt(force / motor_params.n_motors) * motor_params.A + motor_params.B;
    }

  } // namespace quadratic_throttle_model

} // namespace mrs_lib::inline v1

#endif // QUADRATIC_THRUST_MODEL_H
