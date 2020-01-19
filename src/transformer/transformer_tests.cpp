#include <mrs_lib/transformer.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "transformer_tests");
  ros::NodeHandle nh = ros::NodeHandle("~");

  auto tfr = mrs_lib::Transformer("transformer_tests", "uav666", 666);

  return 0;
}

