#include <mrs_lib/math.h>

namespace mrs_lib
{
  /* inv_cdf() method //{ */
  // Inverse cumulative distribution function of a normal probability distribution (aka the probit function)
  // The implementation uses the Beasley-Springer-Moro approximation
  // (see page 68 of Glasserman, Paul, "Monte Carlo Methods in Financial Engineering. Stochastic Modelling and Applied Probability", 2003, doi:10.1007/978-0-387-21617-1)
  // (see https://sci-hub.se/10.1007/978-0-387-21617-1)
  double inv_cdf(const double quantile)
  {
    // polynomial coefficients of the numerator for rational polynomial approximation in the range (0.08; 0.92)
    constexpr double a[4] =
    {
      2.50662823884,
    -18.61500062529,
     41.39119773534,
    -25.44106049637
    };

    // polynomial coefficients of the denominator for the rational polynomial approximation in the range (0.08; 0.92)
    constexpr double b[4] =
    {
      -8.47351093090,
      23.08336743743,
     -21.06224101826,
       3.13082909833
    };

    // polynomial coefficients of the logarithmical approximation in the range (0; 0.08) U (0.92; 1)
    constexpr double c[9] =
    {
      0.3374754822726147,
      0.9761690190917186,
      0.1607979714918209,
      0.0276438810333863,
      0.0038405729373609,
      0.0003951896511919,
      0.0000321767881768,
      0.0000002888167364,
      0.0000003960315187
    };

    if (quantile == 1.0)
      return std::numeric_limits<double>::infinity();
    if (quantile == 0.0)
      return -std::numeric_limits<double>::infinity();
    if (quantile < 0.0 || quantile > 1.0)
      return std::numeric_limits<double>::quiet_NaN();

    const double y = quantile - 0.5;
    if (std::abs(y) < 0.42)
    {
      const double r = y*y;
      const double num = y*((( a[3]*r + a[2] )*r + a[1])*r + a[0]);
      const double denom = (((( b[3]*r + b[2] )*r + b[1])*r + b[0])*r + 1);
      return num/denom;
    }
    else
    {
      const double v = y > 0 ? 1.0 - quantile : quantile;
      const double r = std::log(-std::log(v));
      const double x = c[0] + r*( c[1] + r*( c[2] + r*( c[3] + r*( c[4] + r*( c[5] + r*( c[6] + r*( c[7] + r*c[8] )))))));
      return y < 0 ? -x : x;
    }
  }
  //}
}
