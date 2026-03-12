#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <cmath>
#include <mrs_lib/param_loader.h>

namespace mrs_lib
{

  namespace quadratic_throttle_model
  {

    struct motor_params_t
    {
      int    n_motors;
      double a;
      double b;
      double c;
      enum class type_t
      {
        legacy,
        full_quadratic,
      } type;

      // helper pre-computed constants
      double an;
      double bn;
      double cn;

      motor_params_t() = default;

      motor_params_t(int n_motors, double a, double b)
        : n_motors(n_motors), a(a), b(b), type(type_t::legacy)
      {}

      motor_params_t(int n_motors, double a, double b, double c)
        : n_motors(n_motors), a(a), b(b), c(c), type(type_t::full_quadratic), an(a / n_motors), bn(b / n_motors), cn(c / n_motors)
      {}

      void initialize(mrs_lib::ParamLoader& pl)
      {
        // motor params are not prefixed, since they are common to more nodes
        pl.loadParam<double>("motor_params/a", a);
        pl.loadParam<double>("motor_params/b", b);
        const auto c_loaded = pl.loadParam("motor_params/c", c, 0.0);
        pl.loadParam("motor_params/n_motors", n_motors );

        if (c_loaded)
        {
          type = type_t::full_quadratic;
          an = a/n_motors;
          bn = b/n_motors;
          cn = c/n_motors;
        }
      }
    };

    double inline throttleToForce(const motor_params_t& motor_params, const double throttle)
    {
      // F = N * ( a * T * T + b * T + c )
      if (motor_params.type == motor_params_t::type_t::full_quadratic)
        return motor_params.n_motors * ( motor_params.a*throttle*throttle + motor_params.b*throttle + motor_params.c );
      else
        return motor_params.n_motors * pow((throttle - motor_params.b) / motor_params.a, 2);
    }

    double inline forceToThrottle(const motor_params_t& motor_params, const double force)
    {
      // F = N*( a*T*T + b*T + c )
      // F = a/N * T * T + b/N * T + c/N
      // 0 = a/N * T * T + b/N * T + ( c/N - F )
      // a' = a/N, b' = b/N, c' = c/N - F
      if (motor_params.type == motor_params_t::type_t::full_quadratic)
      {
        const double cp = motor_params.cn - force;
        double sqrt_part = motor_params.bn * motor_params.bn - 4.0 * motor_params.an * cp ;
        // TODO: add some warning here
        if (sqrt_part <= 0.0)
          sqrt_part = 0;
        // only the right half of the parabole is used, so only root1 is necessary
        const double root1 = (- motor_params.bn + std::sqrt( sqrt_part )) / (2 * motor_params.an);
        /* const double root2 = (- motor_params.bn - std::sqrt( motor_params.bn * motor_params.bn - 4.0 * motor_params.an * Cp )) / (2 * motor_params.an); */
        return root1;
      }
      else
        // the legacy computation
        return sqrt(force / motor_params.n_motors) * motor_params.a + motor_params.b;
    }

  }  // namespace quadratic_throttle_model

}  // namespace mrs_lib

#endif  // QUADRATIC_THRUST_MODEL_H
