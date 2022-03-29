// clang: MatousFormat
/**  \file
     \brief Defines useful math functions that are not part of the cmath STD library.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef MATH_H
#define MATH_H

#include <cmath>

namespace mrs_lib
{
  /* probit() function //{ */
  // 
  // 

  /*!
   * \brief Inverse cumulative distribution function of the standard normal probability distribution.
   *
   * Implements the quantile function of a standard normal probability distribution (aka the probit function, see https://en.wikipedia.org/wiki/Probit).
   * The implementation uses the Beasley-Springer-Moro approximation
   * (see page 68 of Glasserman, Paul, "Monte Carlo Methods in Financial Engineering. Stochastic Modelling and Applied Probability", 2003, doi:10.1007/978-0-387-21617-1, available at https://sci-hub.se/10.1007/978-0-387-21617-1).
   *
   * \param quantile the probability that a realization of a random variable with a standard normal dostribution is equal or less than the returned value (see https://en.wikipedia.org/wiki/Quantile).
   *
   * \returns such a value that the probability that a realization of a random variable with a standard normal dostribution is equal or less is \p quantile.
   *
   */
  double probit(const double quantile);

  //}
}

#endif
