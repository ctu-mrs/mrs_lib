#include <mrs_lib/iir_filter.h>

namespace mrs_lib
{

// | ------------------------ IirFilter ------------------------ |

/* IirFilter constructor //{ */

IirFilter::IirFilter(const std::vector<double>& a_in, const std::vector<double>& b_in) {

  /* _a_ = a; */
  /* _b_ = b; */

  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: IIR Filter initialized!");
  /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: a: " << _a_[0] << " " << _a_[1] << " " << _a_[2]); */
  /* ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: b: " << _b_[0] << " " << _b_[1] << " " << _b_[2]); */

  /* size_t a_size = a_in.size(); */
  /* size_t b_size = b_in.size(); */

  /* if (a_size != b_size || a_size == 0) { */
  /*   ROS_ERROR("[%s]: Parameters a and b needs to have the same non-zero length!!", ros::this_node::getName().c_str()); */
  /*   ros::shutdown(); */
  /* } */
  ROS_INFO("pes]: ");
  ROS_INFO("kot]: ");
  a0 = a_in.at(0);
  a1 = a_in.at(0);
  a2 = a_in.at(0);

  b0 = b_in.at(0);
  b1 = b_in.at(0);
  b2 = b_in.at(0);

  _order_ = 3;
  /* input_buffer_.resize(_order_ - 1, 0.0); */
  /* output_buffer_.resize(_order_ - 1, 0.0); */
}

//}

/* iterate() //{ */

double IirFilter::iterate(const double& input) {

  double output;
  output = SectCalcForm1(0, input);
  output = SectCalcForm1(0, output);


  return output;
}

double IirFilter::SectCalcForm1(int k, double x) {
  double y, CenterTap;

  CenterTap = x * b0 + b1 * RegX1 + b2 * RegX2;
  y         = a0 * CenterTap - a1 * RegY1 - a2 * RegY2;

  RegX2 = RegX1;
  RegX1 = x;
  RegY2 = RegY1;
  RegY1 = y;

  return (y);
}
}  // namespace mrs_lib
