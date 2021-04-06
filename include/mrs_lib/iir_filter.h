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


private:
  std::vector<double> a_;
  std::vector<double> b_;
  size_t              order_;
  std::vector<double> buffer_;
};

}  // namespace mrs_lib

#endif
