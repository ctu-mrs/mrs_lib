/**  \file
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include <mrs_lib/iir_filter.h>
#include <mrs_lib/internal/version_macros.hpp>

namespace mrs_lib::MRS_LIB_INTERNAL_INLINE_API_V1 v1
{

  class NotchFilter
  {

  public:
    NotchFilter(const double& sample_rate, const double& frequency_in, const double& bandwidth_in);

    double iterate(double& sample_in);

  private:
    std::unique_ptr<mrs_lib::IirFilter> filter;
  };

} // namespace mrs_lib::inline v1

#endif
