#include <mrs_lib/dynamic_publisher.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("dynamic_publisher_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  mrs_lib::DynamicPublisher dynpub(nh);

  ros::Duration slp(0.5);

  std_msgs::Float32 float_msg;
  float_msg.data = 666.0f;
  std_msgs::String string_msg;

  ROS_INFO("Publishing topics! Use `rostopic list` and `rostopic echo <topic_name>` to see them.");
  while (ros::ok())
  {
    float_msg.data += 666.0f;
    dynpub.publish("float_topic", float_msg);

    string_msg.data = "666";
    dynpub.publish("string_topic", string_msg);

    ros::spinOnce();
    slp.sleep();
  }

  return 0;
}
