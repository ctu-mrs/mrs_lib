/**  \file
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <Eigen/Dense>
#include <ros/ros.h>

#include <mrs_lib/iir_filter.h>

namespace mrs_lib
{

class NotchFilter {

public:
  /* NotchFilter(const double& sample_rate, const std::vector<double>& frequencies_in, const std::vector<double>& bandwidths_in); */
  NotchFilter(const double& sample_rate, const double& frequency_in, const double& bandwidth_in);

  double iterate(double& sample_in);

private:
  std::unique_ptr<mrs_lib::IirFilter> filter;
};

}  // namespace mrs_lib

#endif
