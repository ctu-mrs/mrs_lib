// clang: MatousFormat
/**  \file
     \brief Example file for the SubscribeHandler convenience class
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib subscribe_handler_example`.

     See \ref subscribe_handler/example.cpp.
 */

/**  \example "subscribe_handler/example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib subscribe_handler_example`.
 */

// Include the SubscribeHandler header
#include <mrs_lib/subscribe_handler.h>
#include <impl/subscribe_handler.hpp>
#include <std_msgs/String.h>

void timeout_callback(const ros::Time& last_msg)
{
  ROS_ERROR_STREAM("Have not received a message for " << (ros::Time::now()-last_msg).toSec() << " seconds");
}

void message_callback(const std_msgs::String::ConstPtr msg)
{
  ROS_INFO_STREAM("Received: '" << msg->data << "'.");
}

class SubObject
{
  public:
    void callback_method(const std_msgs::String::ConstPtr msg)
    {
      ROS_INFO_STREAM("Object received: '" << msg->data << "'.");
    }

    void timeout_method(const ros::Time& last_msg)
    {
      ROS_ERROR_STREAM("Object has not received a message for " << (ros::Time::now()-last_msg).toSec() << " seconds");
    }
} sub_obj;

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("subscribe_handler_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  /* name of the topic to be handled */
  const std::string topic_name = "test_topic";
  /* after this duration without receiving messages on the handled topic, the timeout_callback will be called */
  const ros::Duration no_message_timeout = ros::Duration(1.0);

  ROS_INFO("[%s]: Creating SubscribeHandlers.", node_name.c_str());

  mrs_lib::SubscribeHandlerOptions shopts(nh);
  shopts.node_name = node_name;
  shopts.no_message_timeout = no_message_timeout;

  /* This is how a new SubscribeHandler object is initialized. */ 
  mrs_lib::SubscribeHandler<std_msgs::String> handler(
            shopts,
            topic_name,
            timeout_callback,
            message_callback
            );

  /* A variation of the factory method for easier use with objects also exists. */ 
  mrs_lib::construct_object(
            handler,
            shopts,
            topic_name,
            no_message_timeout,
            &SubObject::timeout_method, &sub_obj,
            &SubObject::callback_method, &sub_obj
            );

  /* Type of the message may be accessed by C++11 decltype in case of need */ 
  using message_type = mrs_lib::message_type<decltype(handler)>;
  ros::Publisher pub = nh.advertise<message_type>(topic_name, 5);

  /* Now let's just spin to process calbacks until the user decides to stop the program. */ 
  ros::Duration d(3.0);
  while (ros::ok())
  {
    message_type msg;
    msg.data = "asdf";
    pub.publish(msg);
    ROS_INFO_THROTTLE(1.0, "[%s]: Spinning", ros::this_node::getName().c_str());
    ros::spinOnce();
    d.sleep();
  }
}
