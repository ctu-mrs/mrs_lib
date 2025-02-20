/**  \file
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <vector>

namespace mrs_lib
{

class IirFilter {

public:
  /**
   * @brief Constructor of new IirFilter object (coefficients are in the canonical form)
   *
   * @param a gains in the feedback branch of the IIR filter be advised that the coefficient a[0] is not used and should be set to zero.
   * @param b gains in the feedforward branch, using only this coefficients results in FIR filter
   */
  IirFilter(const std::vector<double>& a, const std::vector<double>& b);
  IirFilter();

  double iterate(const double input);

  std::tuple<std::vector<double>, std::vector<double>> getCoeffs();
  std::vector<double>                                  getBuffer();

private:
  std::vector<double> a_;
  std::vector<double> b_;
  size_t              order_;
  std::vector<double> buffer_;
};

}  // namespace mrs_lib

#endif
