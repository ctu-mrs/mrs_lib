#include <mrs_lib/transformer.h>
#include <geometry_msgs/PointStamped.h>

using test_t = geometry_msgs::PointStamped;
template std::optional<test_t> mrs_lib::Transformer::transform<test_t>(const mrs_lib::TransformStamped& to_frame, const test_t& what);
template std::optional<test_t> mrs_lib::Transformer::transformSingle<test_t>(const std::string& to_frame, const test_t& what);

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "transformer_tests");
  ros::NodeHandle nh = ros::NodeHandle("~");

  auto tfr = mrs_lib::Transformer("transformer_tests", "uav666");

  return 0;
}

