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

  auto tfr = mrs_lib::Transformer("transformer_tests", "uav62");
  std::optional<mrs_lib::TransformStamped> tf_opt;
  for (int it = 0; it < 10000 && ros::ok(); it++)
  {
    tf_opt = tfr.getTransform("fcu", "local_origin");
    if (tf_opt.has_value())
      break;
    ros::Duration(0.1).sleep();
  }
  if (tf_opt.has_value())
  {
    auto tf = tf_opt.value();
    std::cout << "from: " << tf.from() << ", to: " << tf.to() << ", stamp: " << tf.stamp() << std::endl;
    std::cout << tf.getTransform() << std::endl;
    auto tf_inv = tf.inverse();
    std::cout << "from: " << tf_inv.from() << ", to: " << tf_inv.to() << ", stamp: " << tf_inv.stamp() << std::endl;
    std::cout << tf_inv.getTransform() << std::endl;
  }


  return 0;
}

