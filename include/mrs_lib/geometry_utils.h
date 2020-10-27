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
      cyclic(const flt val) : val(wrap(val)) {};
      cyclic& operator=(const flt nval) {val = wrap(nval);};
      cyclic& operator=(const cyclic& other) {val = other.val;};

      const flt val;

      static constexpr flt minimum = 0;
      static constexpr flt supremum = 2*M_PI;
      static constexpr flt range = supremum - minimum;
      static constexpr flt half_range = range/flt(2);

      static_assert((supremum > minimum), "cyclic value: Range not valid");   

      static flt inRange(const flt val)
      {
        return val >= minimum && val < supremum;
      }

      static flt wrap(const flt val)
      {
        const flt rem = std::fmod(val - minimum, range);
        const flt wrapped = rem + minimum + std::signbit(rem)*range;
        return wrapped;
      }

      static flt unwrap(const flt from, const flt to)
      {
        return from + diff(to, from);
      }

      static flt pdist(const flt from, const flt to)
      {
        const flt wfrom = wrap(from);
        const flt wto = wrap(to);
        const flt tmp = wto - wfrom;
        const flt dist = tmp + std::signbit(tmp)*range;
        return dist;
      }

      static cyclic pdist(const cyclic from, const cyclic to)
      {
        const flt tmp = to.val - from.val;
        const flt dist = tmp + std::signbit(tmp)*range;
        return dist;
      }

      static flt diff(const flt minuend, const flt subtrahend)
      {
        const flt wminuend = wrap(minuend);
        const flt wsubtrahend = wrap(subtrahend);
        const double d = wminuend - wsubtrahend;
        if (d < -half_range)
          return d + range;
        if (d >= half_range)
          return d - range;
        return d;
      }

      static cyclic diff(const cyclic minuend, const cyclic subtrahend)
      {
        const double d = minuend.value - subtrahend.value;
        if (d < -half_range)
          return d + range;
        if (d >= half_range)
          return d - range;
        return d;
      }

      static flt dist(const flt from, const flt to)
      {
        return std::abs(diff(from, to));
      }

      static cyclic dist(const cyclic from, const cyclic to)
      {
        return std::abs(diff(from, to));
      }

      static flt interp_unwrapped(const flt from, const flt to, const flt coeff)
      {
        const flt diff = diff(to, from);
        const flt intp = from + coeff*diff;
        return intp;
      }

      static cyclic interp_unwrapped(const cyclic from, const cyclic to, const flt coeff)
      {
        const cyclic diff = diff(to, from);
        const cyclic intp = from + coeff*diff;
        return intp;
      }

      static flt interp(const flt from, const flt to, const flt coeff)
      {
        return wrap(interp_unwrapped(from, to, coeff));
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
