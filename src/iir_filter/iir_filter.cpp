#include <mrs_lib/iir_filter.h>

namespace mrs_lib
{

// | ------------------------ IirFilter ------------------------ |

/* IirFilter constructor //{ */

IirFilter::IirFilter(const std::vector<double>& a_in, const std::vector<double>& b_in) {

  a_ = a_in;
  b_ = b_in;

  order_ = a_.size();
  buffer_.resize(order_, 0.0);
  for (size_t i = 0; i < a_.size(); i++) {
    
  ROS_INFO_STREAM("a: " << a_[i] << " b: " << b_[i]);
  }
  ROS_INFO("[%s]: IIR filter initialized!", ros::this_node::getName().c_str());
}

//}

/* iterate() //{ */

double IirFilter::iterate(const double& input) {

  double output = 0;

  buffer_[0] = input;

  for (size_t i = 1; i < order_; i++) {
    buffer_[0] += (-a_[i]) * buffer_[i];
    output += (b_[i]) * buffer_[i];
  }

  output += buffer_[0] * b_[0];

  for (size_t i = order_ - 1; i > 0; i--) {
    buffer_[i] = buffer_[i - 1];
  }

  return output;
}

}  // namespace mrs_lib
