#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <cmath>

namespace mrs_lib
{

  namespace quadratic_throttle_model
  {

    struct motor_params_t
    {
      int    n_motors;
      double A;
      double B;
      double C;
      enum class type_t
      {
        legacy,
        full_quadratic,
      } type;

      // helper pre-computed constants
      double An;
      double Bn;
      double Cn;

      motor_params_t(int n_motors, double A, double B)
        : n_motors(n_motors), A(A), B(B), type(type_t::legacy)
      {}

      motor_params_t(int n_motors, double A, double B, double C)
        : n_motors(n_motors), A(A), B(B), C(C), type(type_t::legacy), An(A / n_motors), Bn(B / n_motors), Cn(C / n_motors)
      {}
    };

    double inline throttleToForce(const motor_params_t& motor_params, const double throttle)
    {
      // F = N * ( A * T * T + B * T + C )
      if (motor_params.type == motor_params_t::type_t::full_quadratic)
        return motor_params.n_motors * ( motor_params.A*throttle*throttle + motor_params.B*throttle + motor_params.C );
      else
        return motor_params.n_motors * pow((throttle - motor_params.B) / motor_params.A, 2);
    }

    double inline forceToThrottle(const motor_params_t& motor_params, const double force)
    {
      // F = N*( A*T*T + B*T + C )
      // F = A/N * T * T + B/N * T + C/N
      // 0 = A/N * T * T + B/N * T + ( C/N - F )
      // A' = A/N, B' = B/N, C' = C/N - F
      if (motor_params.type == motor_params_t::type_t::full_quadratic)
      {
        const double Cp = motor_params.Cn - force;
        double sqrt_part = motor_params.Bn * motor_params.Bn - 4.0 * motor_params.An * Cp ;
        // TODO: add some warning here
        if (sqrt_part <= 0.0)
          sqrt_part = 0;
        // only the right half of the parabole is used, so only root1 is necessary
        const double root1 = (- motor_params.Bn + std::sqrt( sqrt_part )) / (2 * motor_params.An);
        /* const double root2 = (- motor_params.Bn - std::sqrt( motor_params.Bn * motor_params.Bn - 4.0 * motor_params.An * Cp )) / (2 * motor_params.An); */
        return root1;
      }
      else
        // the legacy computation
        return sqrt(force / motor_params.n_motors) * motor_params.A + motor_params.B;
    }

  }  // namespace quadratic_throttle_model

}  // namespace mrs_lib

#endif  // QUADRATIC_THRUST_MODEL_H
