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

    /**
    * \brief Implementation of the a general cyclic value (such as angles in radians/degrees etc).
    *
    * This class implements a periodical value with a period of \f$ r \in {\rm I\!R} \f$ so that a general number \f$ v \in {\rm I\!R} \f$ represents the same value as \f$ v + kr,~k \in {\rm I\!N} \f$.
    * For the purposes of calculations in this class, \f$ v \f$ is confined to a half-open interval \f$ v \in [~m,~s~[ \f$, where \f$ m \f$ is the minimum of this interval and \f$ s \f$ is its supremum, and \f$ s = m + r \f$ holds.
    * This approach enables representing \f$ v \f$ in different intervals on the real numbers axis; eg. angle in radians may be represented within the interval \f$ v \in [~-\pi,~\pi~[ \f$ or \f$ v \in [~0,~2\pi~[ \f$, according to the needs of the specific application.
    * The period \f$ r \f$ is called \p range for the purposes of this class, as it represents range of the interval of valid ("wrapped") values of \f$ v \f$.
    *
    * This class may be used as an object or its static methods may be used on regular floating-point types, avoiding any object-related overheads (see example).
    * Specializations for the most common cyclic values are provided and a new specialization may be easily created simply by inheriting from this class and specifying a different minimum and supremum values.
    *
    * \parblock
    * \note For a better intuitive understanding of the used functions, the term **walk** is sometimes used in the function explanations.
    * You can imagine walking along the circle from one angle to another (represented by the circular quantities).
    * The walk may be the shortest - then you're walking in such a manner that you reach the other point in the least of steps.
    * The walk may also be oriented - then you're walking in a specific direction (ie. according to the increasing/decreasing angle).
    * \endparblock
    *
    * \parblock
    * \note The terms **circular quantity** and **value** are used in the function explanations.
    * A circular quantity is eg. an angle and the same quantity may be represented using different values: the same angle is represented by the values of 100 degrees and 460 degrees.
    * \endparblock
    *
    * \tparam flt floating data type to be used by this class.
    *
    */
    template <typename flt, class spec>
    struct cyclic
    {
  /*!
    * \brief Default constructor.
    *
    * Sets the value to the minimum.
    */
      cyclic() : val(minimum) {};
  /*!
    * \brief Constructor overload.
    *
    * \param val initialization value (will be wrapped).
    */
      cyclic(const flt val) : val(wrap(val)) {};
  /*!
    * \brief Copy constructor.
    *
    * \param val initialization value.
    */
      cyclic(const cyclic& other) : val(other.val) {};
  /*!
    * \brief Assignment operator.
    *
    * \param nval value to be assigned (will be wrapped).
    * \return     reference to self.
    */
      cyclic& operator=(const flt nval) {val = wrap(nval); return *this;};
  /*!
    * \brief Assignment operator.
    *
    * \param other value to be assigned.
    * \return      reference to self.
    */
      cyclic& operator=(const cyclic& other) {val = other.val; return *this;};
  /*!
    * \brief Getter for \p val.
    *
    * \return the value.
    */
      flt value() const {return val;};

      static constexpr flt minimum = spec::minimum;     /*!< \brief Minimum of the valid interval of wrapped values \f$ m \f$ */
      static constexpr flt supremum = spec::supremum;   /*!< \brief Supremum of the valid interval of wrapped values \f$ s \f$ */
      static constexpr flt range = supremum - minimum;  /*!< \brief Range of the valid interval of wrapped values \f$ r \f$ (also the period of the cyclic quantity). */
      static constexpr flt half_range = range/flt(2);   /*!< \brief Half of the range of the valid interval of wrapped values \f$ r/2 \f$ (used for some calculations). */

      /* static_assert((supremum > minimum), "cyclic value: Range not valid"); */   

  /*!
    * \brief Checks if \p val is within the valid interval of wrapped values.
    *
    * \param val the value to be checked.
    * \returns   true if \p val is within the valid interval of wrapped values.
    */
      static bool inRange(const flt val)
      {
        return val >= minimum && val < supremum;
      }

  /*!
    * \brief Returns \p val, converted to the valid interval of values.
    *
    * The wrapped value represents the same quantity as the parameter (ie. \f$ v' = v + kr \f$, where \f$ v \f$ is the parameter and \f$ v' \f$ is the returned value).
    *
    * \param val the value to be wrapped.
    * \returns   \p val wrapped to the valid interval of values.
    */
      static flt wrap(const flt val)
      {
        // these few ifs should cover most cases, improving speed and precision
        if (val >= minimum)
        {
          if (val < supremum) // value is actually in range and doesn't need to be wrapped
            return val;
          else if (val < supremum + range)
            return val - range; // to avoid unnecessary costly fmod operation for this case (assumed to be significantly more common than the general case)
        }
        else
        {
          if (val >= minimum - range)
            return val + range; // to avoid unnecessary costly fmod operation for this case (assumed to be significantly more common than the general case)
        }

        // general case
        const flt rem = std::fmod(val - minimum, range);
        const flt wrapped = rem + minimum + std::signbit(rem)*range;
        return wrapped;
      }

  /*!
    * \brief Returns value of the parameter \p to modified so that there is no "jump" between \p from and \t to.
    *
    * The circular difference between the two input quantities is preserved, but the returned value is modified if necessary so that the linear distance between the values is the smallest.
    * This is useful whenever you need to preserve linear continuity of consecutive values, eg. when commanding a multi-rotational servo motor or when using a simple linear Kalman filter to estimate circular quantities.
    *
    * An example of inputs and outputs if \f$ m = 0,~s=360 \f$:
    *  \p from  | \p to | \p return
    *  -------- | ------| ---------
    *     10    |   20  |   20
    *     350   |   20  |   380
    *     350   |   0   |   360
    *     10    |   200 |  -160
    *
    * \param from   the previous value from which the unwrapped value of \p to should have the same circular difference and minimal linear distance.
    * \param to     the value to be unwrapped.
    * \returns      the unwrapped value of \p to.
    *
    * \warning Note that the returned value may be outside the valid interval of wrapped values, specified by the \p minimum and \p supremum parameters of this class.
    */
      static flt unwrap(const flt from, const flt to)
      {
        return from + diff(to, from);
      }

  /*!
    * \brief Returns length of the shortest walk in the positive direction from the first parameter to the second one.
    *
    * \param from   the positive walk starts at this value.
    * \param to     the positive walk ends at this value.
    * \returns      length of the shortest positive walk from the first parameter to the second one.
    *
    * \note The returned value is not necessarily the shortest distance between the two circular quantities (see the dist() function for that).
    */
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

  /*!
    * \brief Returns the difference between the two circular values.
    *
    * The difference may also be interpreted as length of the shortest walk between the two values, with a sign according to the direction of the shortest walk.
    *
    * \param minuend      the \p subtrahend will be subtracted from this value.
    * \param subtrahend   this value will be subtracted from the \p minuend.
    * \returns            the difference of the two circular quantities.
    */
      static flt diff(const flt minuend, const flt subtrahend)
      {
        const flt wminuend = wrap(minuend);
        const flt wsubtrahend = wrap(subtrahend);
        const flt d = wminuend - wsubtrahend;
        if (d < -half_range)
          return d + range;
        if (d >= half_range)
          return d - range;
        return d;
      }

      static flt diff(const cyclic minuend, const cyclic subtrahend)
      {
        const flt d = minuend.value - subtrahend.value;
        if (d < -half_range)
          return d + range;
        if (d >= half_range)
          return d - range;
        return d;
      }

  /*!
    * \brief Returns the distane between the two circular values.
    *
    * The distance may also be interpreted as length of the shortest walk between the two values.
    *
    * \param from  the first circular quantity.
    * \param to    the second circular quantity.
    * \returns     distance of the two circular quantities.
    *
    * \note The order of the parameters doesn't matter.
    */
      static flt dist(const flt from, const flt to)
      {
        return std::abs(diff(from, to));
      }

      static cyclic dist(const cyclic from, const cyclic to)
      {
        return std::abs(diff(from, to));
      }

  /*!
    * \brief Interpolation between two circular quantities without wrapping of the result.
    *
    * This function doesn't wrap the returned value.
    *
    * \param from  the first circular quantity.
    * \param to    the second circular quantity.
    * \param coeff the interpolation coefficient.
    * \returns     interpolation of the two circular quantities using the coefficient.
    *
    * \warning Note that the returned value may be outside the valid interval of wrapped values, specified by the \p minimum and \p supremum parameters of this class.
    */
      static flt interpUnwrapped(const flt from, const flt to, const flt coeff)
      {
        const flt diff = diff(to, from);
        const flt intp = from + coeff*diff;
        return intp;
      }

      static flt interpUnwrapped(const cyclic from, const cyclic to, const flt coeff)
      {
        const flt diff = diff(to, from);
        const flt intp = from + coeff*diff;
        return intp;
      }

  /*!
    * \brief Interpolation between two circular quantities without wrapping of the result.
    *
    * This function wraps the returned value so that it is in the interval of valid values.
    *
    * \param from  the first circular quantity.
    * \param to    the second circular quantity.
    * \param coeff the interpolation coefficient.
    * \returns     interpolation of the two circular quantities using the coefficient, wrapped to the interval of valid values.
    */
      static flt interp(const flt from, const flt to, const flt coeff)
      {
        return wrap(interp_unwrapped(from, to, coeff));
      }

      static cyclic interp(const cyclic from, const cyclic to, const flt coeff)
      {
        return interp_unwrapped(from, to, coeff);
      }

  /*!
    * \brief Conversion between two different circular quantities.
    *
    * This function converts its parameter, interpreted as the circular quantity, represented by this class, to the \p other_t type of circular quantity.
    *
    * \param what       the circular quantity to be converted.
    * \returns          the circular quantity converted to the range of \p other_t.
    * \tparam other_t   type of the circular quantity to be converted to.
    *
    * \warning For the purposes of this function, it is assumed that the range of one type corresponds to the whole range of the other type and zeros of both types correspond to each other (such as when converting eg. degrees to radians).
    */
      template <class other_t>
      static other_t convert(const cyclic& what)
      {
        return other_t(what.val/range*other_t::range);
      };

  /*!
    * \brief Conversion between two different circular quantities.
    *
    * This method returns the circular quantity, represented by this object, converted to the \p other_t type of circular quantity.
    *
    * \returns          the circular quantity converted to the range of \p other_t.
    * \tparam other_t   type of the circular quantity to be converted to.
    *
    * \warning For the purposes of this function, it is assumed that the range of one type corresponds to the whole range of the other type and zeros of both types correspond to each other (such as when converting eg. degrees to radians).
    */
      template <class other_t>
      other_t convert() const
      {
        return other_t(val/range*other_t::range);
      };

    protected:
      flt val;
    };

    struct radians : public cyclic<double, radians>
    {
      using cyclic<double, radians>::cyclic; // necessary to inherit constructors
      static constexpr double minimum = 0;
      static constexpr double supremum = 2*M_PI;
    };

    struct sradians : public cyclic<double, sradians>
    {
      using cyclic<double, sradians>::cyclic; // necessary to inherit constructors
      static constexpr double minimum = -M_PI;
      static constexpr double supremum = M_PI;
    };

  }  // namespace geometry
}  // namespace mrs_lib

#endif
