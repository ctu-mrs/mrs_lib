/**  \file
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <ros/ros.h>

namespace mrs_lib
{

class IirFilter {

public:
  IirFilter(const std::vector<double>& a, const std::vector<double>& b);

  double iterate(const double& input);
  double SectCalcForm1(int k, double x);


private:
  std::vector<double> _a_;
  std::vector<double> _b_;
  size_t              _order_;
  std::vector<double> input_buffer_;
  std::vector<double> output_buffer_;

  double RegX1 = 0.0;
  double RegX2 = 0.0;
  double RegY1 = 0.0;
  double RegY2 = 0.0;

  double a0 = 0.0;
  double a1 = 0.0;
  double a2 = 0.0;

  double b0 = 0.0;
  double b1 = 0.0;
  double b2 = 0.0;
};

}  // namespace mrs_lib

#endif
