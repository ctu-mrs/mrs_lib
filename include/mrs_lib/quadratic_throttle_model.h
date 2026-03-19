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
      motor_params_t() = default;

      motor_params_t(int n_motors, double a, double b) : n_motors(n_motors), a(a), b(b), c(0), type(type_t::legacy), an(0), bn(0), cn(0)
      {
      }

      motor_params_t(int n_motors, double a, double b, double c)
          : n_motors(n_motors), a(a), b(b), c(c), type(type_t::full_quadratic), an(a * n_motors), bn(b * n_motors), cn(c * n_motors)
      {
        if (a < 1e-6)
          type = type_t::linear;
      }

      void initialize(mrs_lib::ParamLoader& pl)
      {
        // motor params are not prefixed, since they are common to more nodes
        pl.loadParam<double>("motor_params/a", a);
        pl.loadParam<double>("motor_params/b", b);
        type = type_t::legacy;
        const auto c_loaded = pl.loadParam("motor_params/c", c, 0.0);
        pl.loadParam("motor_params/n_motors", n_motors);

        if (c_loaded)
        {
          type = type_t::full_quadratic;
          an = a * n_motors;
          bn = b * n_motors;
          cn = c * n_motors;

          if (a < 1e-6)
            type = type_t::linear;
        }
      }

      int n_motors;
      double a;
      double b;
      double c;
      enum class type_t
      {
        legacy,
        full_quadratic,
        linear,
      } type;

      // helper pre-computed constants
      double an;
      double bn;
      double cn;
    };

    double inline throttleToForce(const motor_params_t& motor_params, const double throttle)
    {
      if (motor_params.type == motor_params_t::type_t::full_quadratic || motor_params.type == motor_params_t::type_t::linear)
        // F = N*( a*T^2 + b*T + c )
        return motor_params.n_motors * (motor_params.a * throttle * throttle + motor_params.b * throttle + motor_params.c);
      else
        // F = N*( (T - b)/a )^2
        return motor_params.n_motors * pow((throttle - motor_params.b) / motor_params.a, 2);
    }

    double inline forceToThrottle(const motor_params_t& motor_params, const double force)
    {
      if (motor_params.type == motor_params_t::type_t::linear || motor_params.a < 1e-6)
      {
        // F = N*( b*T + c )
        // F = b*N*T + c*N
        // 0 = b*N*T + c*N - F
        // T = ( F - c*N ) / ( b*N )
        if (motor_params.bn < 1e-6)
        {
          // RCLCPP_ERROR_THROTTLE(1.0, "forceToThrottle: The N*b parameter is too small!");
          return 0.0;
        }
        return (force - motor_params.cn) / motor_params.bn;
      } else if (motor_params.type == motor_params_t::type_t::full_quadratic)
      {
        // F = N*( a*T^2 + b*T + c )
        // F = a*N*^2 + b*N*T + c*N
        // 0 = a*N*T^2 + b*N*T + ( c*N - F )
        // a' = a*N, b' = b*N, c' = c*N - F

        const double cp = motor_params.cn - force;
        double discriminant = motor_params.bn * motor_params.bn - 4.0 * motor_params.an * cp;
        // TODO: add some warning here
        if (discriminant < 0.0)
        {
          // RCLCPP_ERROR_THROTTLE(1.0, "forceToThrottle: Discriminant of throttle solution is negative, setting zero!");
          discriminant = 0;
        }
        // only the right half of the parabole is used, so only root1 is necessary
        const double root1 = (-motor_params.bn + std::sqrt(discriminant)) / (2 * motor_params.an);
        /* const double root2 = (- motor_params.bn - std::sqrt( discriminant )) / (2 * motor_params.an); */
        return std::clamp(root1, 0.0, 1.0);
      } else
        // the legacy computation
        return sqrt(force / motor_params.n_motors) * motor_params.a + motor_params.b;
    }

  } // namespace quadratic_throttle_model

} // namespace mrs_lib

#endif // QUADRATIC_THRUST_MODEL_H
