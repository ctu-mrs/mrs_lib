#include <mrs_lib/iir_filter.h>

namespace mrs_lib
{

// | ------------------------ IirFilter ------------------------ |

/* IirFilter constructor //{ */

IirFilter::IirFilter(const std::vector<double>& a_in, const std::vector<double>& b_in) {

  a_ = a_in;
  b_ = b_in;

  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: IIR Filter initialized!");
  /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: a: " << _a_[0] << " " << _a_[1] << " " << _a_[2]); */
  /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: b: " << _b_[0] << " " << _b_[1] << " " << _b_[2]); */

  /* size_t a_size = a_in.size(); */
  /* size_t b_size = b_in.size(); */

  /* if (a_size != b_size || a_size == 0) { */
  /*   ROS_ERROR("[%s]: Parameters a and b needs to have the same non-zero length!!", ros::this_node::getName().c_str()); */
  /*   ros::shutdown(); */
  /* } */
  /* a0 = a_in.at(0); */
  /* a1 = a_in.at(1); */
  /* a2 = a_in.at(2); */

  /* b0 = b_in.at(0); */
  /* b1 = b_in.at(1); */
  /* b2 = b_in.at(2); */


  order_ = a_.size();
  buffer_.resize(order_, 0.0);
}

//}

/* iterate() //{ */

double IirFilter::iterate(const double input) {

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
