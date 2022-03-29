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
  /* inv_cdf() method //{ */
  // 
  // 

  /*!
   * \brief Returns the heading angle from the rotation.
   *
   * Inverse cumulative distribution function of a standard normal probability distribution (aka the probit function, see https://en.wikipedia.org/wiki/Probit)
   * The implementation uses the Beasley-Springer-Moro approximation
   * (see page 68 of Glasserman, Paul, "Monte Carlo Methods in Financial Engineering. Stochastic Modelling and Applied Probability", 2003, doi:10.1007/978-0-387-21617-1
   * (see https://sci-hub.se/10.1007/978-0-387-21617-1)).
   *
   * \param quantile probability that a realization of a random variable with a standard normal dostribution is equal or less than the returned value.
   *
   * \returns such a value that the probability that a realization of a random variable with a standard normal dostribution is equal or less is \p quantile.
   *
   */
  double inv_cdf(const double quantile);

  //}
}

#endif
