// clang: MatousFormat
/**  \file
     \brief Example file for the SubscribeHandler convenience class
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     A simple example demonstrating basic usage of the SubscribeHandler class.
     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib subscribe_handler_simple_example`.

     See \ref subscriber_handler/simple_example.cpp.
 */

/**  \example "subscriber_handler/simple_example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib subscribe_handler_simple_example`.
 */

// Include the SubscribeHandler header
#include <mrs_lib/subscribe_handler.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name = "subscribe_handler_simple_example";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  /* Basic configuration of the SubscribeHandler. */
  mrs_lib::SubscribeHandlerOptions shopts(nh);
  shopts.node_name = node_name;                       //< used for the ROS logging from inside the SubscribeHandler (ROS_INFO, ROS_WARN etc.)
  shopts.no_message_timeout = ros::Duration(3.0);     //< SubscribeHandler warns if no messages arrive for this duration

  /* name of the topic to be handled */
  const std::string topic_name = "test_topic";

  /* This is how a new SubscribeHandler object is initialized. */ 
  mrs_lib::SubscribeHandler<std_msgs::String> handler(
            shopts,
            topic_name
            );

  /* Now let's just spin to process calbacks until the user decides to stop the program. */ 
  ros::Duration d(0.1);
  while (ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "[" << node_name << "]: Spinning");
    ros::spinOnce();
    /* Print out any new messages */
    if (handler.newMsg())
      ROS_INFO_STREAM("[" << node_name << "]: Got a new message: " << handler.getMsg()->data);
    d.sleep();
  }
}

