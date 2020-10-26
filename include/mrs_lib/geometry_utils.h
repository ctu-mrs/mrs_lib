// clang: MatousFormat
/**  \file
     \brief Defines useful geometry utilities and functions.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Petr Štibinger - stibipet@fel.cvut.cz
 */

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <vector>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>

#include <mrs_lib/utils.h>

#include <iostream>

namespace mrs_lib
{
  namespace geometry
  {
    template <int dims>
    using vec_t = Eigen::Matrix<double, dims, 1>;

    using pt2_t = vec_t<2>;
    using vec2_t = vec_t<2>;
    using pt3_t = vec_t<3>;
    using vec3_t = vec_t<3>;

    template <int dims>
    vec_t<dims + 1> to_homogenous(const vec_t<dims>& vec)
    {
      const Eigen::Matrix<double, dims + 1, 1> ret((Eigen::Matrix<double, dims + 1, 1>() << vec, 1).finished());
      return ret;
    }

    template <typename flt>
    struct cyclic
    {
      cyclic() = delete;

      static constexpr flt minimum = 0;
      static constexpr flt supremum = 2*M_PI;
      static constexpr flt range = supremum - minimum;
      static constexpr flt half_range = range/flt(2);

      static_assert((supremum > minimum), "cyclic: Range not valid");   

      static flt inRange(const flt a)
      {
        return a >= minimum && a < supremum;
      }

      static flt wrap(const flt a)
      {
        const flt rem = std::fmod(a - minimum, range);
        const flt wrapped = rem + minimum + std::signbit(rem)*range;
        return wrapped;
      }

      static flt dist(const flt a, const flt b)
      {
        const flt tmp = b - a;
        const flt dist = tmp + std::signbit(tmp)*range;
        return dist;
      }

      static flt diff(const flt a, const flt b)
      {
        const double d = a-b;
        if (d < -half_range)
          return d + range;
        if (d >= half_range)
          return d - range;
        return d;
      }
    };

    struct radians : public cyclic<double>
    {
      static constexpr double minimum = 0;
      static constexpr double supremum = 2*M_PI;
    };

    struct sradians : public cyclic<double>
    {
      static constexpr double minimum = -M_PI;
      static constexpr double supremum = M_PI;
    };

  }  // namespace geometry
}  // namespace mrs_lib

#endif
