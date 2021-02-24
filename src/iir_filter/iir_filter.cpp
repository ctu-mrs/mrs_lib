#include <mrs_lib/iir_filter.h>

namespace mrs_lib
{

// | ------------------------ IirFilter ------------------------ |

/* IirFilter constructor //{ */

IirFilter::IirFilter(const std::vector<double>& a, const std::vector<double>& b) {

  _a_ = a;
  _b_ = b;

  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: IIR Filter initialized!");
  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: a: " << _a_[0] << " " << _a_[1] << " " << _a_[2]);
  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: b: " << _b_[0] << " " << _b_[1] << " " << _b_[2]);

  size_t a_size = a.size();
  size_t b_size = b.size();
  
  if (a_size != b_size || a_size != 0) {
   ROS_ERROR("[%s]: Parameters a and b needs to have the same non-zero length!!", ros::this_node::getName().c_str()); 
   ros::shutdown();
  }

  _order_ = a_size;
  buffer_.resize(_order_, 0.0);

}

//}

/* iterate() //{ */

double IirFilter::iterate(const double& input)
{
  double buffer_input = input*_a_[0];

  for (size_t i = 1; i < _order_; i++) {
    buffer_input -= buffer_[i]*_a_[i];
  }

  for (size_t i = 1; i < _order_; i++) {
    buffer_[i] = buffer_[i-1];
  }

  buffer_[0] = buffer_input;

  double output = 0;

  for (size_t i = 0; i < _order_; i++) {
    output += buffer_[i]*_b_[i];
  }

  return output;
}

//}

}  // namespace mrs_lib
