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
#include <std_msgs/String.h>

void timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
{
  ROS_ERROR_STREAM("Have not received message from topic '" << topic << "' for " << (ros::Time::now()-last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");
}

void message_callback(mrs_lib::SubscribeHandler<std_msgs::String>& sh)
{
  ROS_INFO_STREAM("Received: '" << sh.getMsg()->data << "' from topic '" << sh.topicName() << "'");
}

class SubObject
{
  public:
    void callback_method(mrs_lib::SubscribeHandler<std_msgs::String>& sh)
    {
      ROS_INFO_STREAM("Object received: '" << sh.getMsg()->data << "' from topic '" << sh.topicName() << "'");
    }

    void timeout_method(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
    {
      ROS_ERROR_STREAM("Object has not received message from topic '" << topic << "' for " << (ros::Time::now()-last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");
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
  /* whether mutexes should be used to prevent data races (set to true in a multithreaded scenario such as nodelets) */
  const bool threadsafe = false;

  ROS_INFO("[%s]: Creating SubscribeHandlers.", node_name.c_str());

  mrs_lib::SubscribeHandlerOptions shopts(nh);
  shopts.node_name = node_name;
  shopts.threadsafe = threadsafe;

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
            /* timeout_callback, */
            /* message_callback */
            &SubObject::timeout_method, &sub_obj,
            &SubObject::callback_method, &sub_obj
            );

  /* A variation of the factory method for easier use with objects also exists. */ 
  mrs_lib::construct_object(
            handler,
            shopts,
            topic_name,
            no_message_timeout,
            /* timeout_callback, */
            /* message_callback */
            &SubObject::callback_method, &sub_obj,
            &SubObject::timeout_method, &sub_obj
            );

  /* Type of the message may be accessed by C++11 decltype in case of need */ 
  using message_type = mrs_lib::message_type<decltype(handler)>;
  ros::Publisher pub = nh.advertise<message_type>(topic_name, 5);

  /* Now let's just spin to process calbacks until the user decides to stop the program. */ 
  ros::Rate r(2);
  while (ros::ok())
  {
    message_type msg;
    msg.data = "asdf";
    pub.publish(msg);
    ROS_INFO_THROTTLE(1.0, "[%s]: Spinning", ros::this_node::getName().c_str());
    ros::spinOnce();
    r.sleep();
  }
}
