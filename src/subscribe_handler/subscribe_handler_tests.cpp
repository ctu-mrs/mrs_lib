#include <std_msgs/Int64.h>
/* #include <geometry_msgs/PointStamped.h> */
#include <mrs_lib/subscribe_handler.h>

using message_type = std_msgs::Int64;
/* using message_type = geometry_msgs::PointStamped; */

void timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
{
  ROS_ERROR_STREAM("Have not received message from topic '" << topic << "' for " << (ros::Time::now()-last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");
}

void message_callback(mrs_lib::SubscribeHandlerPtr<message_type> sh_ptr)
{
  ROS_INFO_STREAM("Received '" << sh_ptr->get_data() << "'");
  /* ROS_INFO_STREAM("Received: '" << (int)sh_ptr->get_data()->data << "'"); */
}

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("subscribe_handler_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  /* name of the topic to be handled */
  const std::string topic_name = "test_topic";
  /* after this duration without receiving messages on the handled topic, the timeout_callback will be called */
  const ros::Duration no_message_timeout = ros::Duration(2.0);
  /* whether mutexes should be used to prevent data races (set to true in a multithreaded scenario such as nodelets) */
  const bool threadsafe = false;
  const bool autostart = false;
  const bool time_consistent = false;
  const uint32_t queue_size = 5;
  const ros::TransportHints& transport_hints = ros::TransportHints();

  ROS_INFO("[%s]: Creating SubscribeHandlers using SubscribeMgr.", node_name.c_str());
  mrs_lib::SubscribeMgr smgr(nh);

  /* This is how a new SubscribeHandler object is initialized. */ 
  auto handler1 = smgr.create_handler<message_type, time_consistent>(
            topic_name,
            no_message_timeout,
            timeout_callback,
            message_callback,
            threadsafe,
            autostart,
            queue_size,
            transport_hints
            );

  /* Type of the message may be accessed by C++11 decltype in case of need */ 
  /* using message_type = std::remove_const<decltype(handler1)::element_type::message_type>::type; */
  ros::Publisher pub = nh.advertise<message_type>(topic_name, 5);

  /* Now let's just spin to process calbacks until the user decides to stop the program. */ 
  ros::Rate r(1);
  int n = 0;
  message_type msg;
  while (ros::ok())
  {
    if (n % 10 == 0)
    {
      ROS_INFO("[%s]: STARTING handler", ros::this_node::getName().c_str());
      handler1->start();
    } else if (n % 5 == 0)
    {
      ROS_INFO("[%s]: STOPPING handler", ros::this_node::getName().c_str());
      handler1->stop();
    }

    /* msg.header.stamp = ros::Time::now(); */
    /* msg.point.x = msg.point.y = msg.point.z = n; */
    msg.data = n;
    ROS_INFO_STREAM("Publishing message '" << msg << "'");
    /* ROS_INFO("[%s]: Publishing message '%ld'", ros::this_node::getName().c_str(), msg.data); */
    pub.publish(msg);

    ROS_INFO("[%s]: Spinning", ros::this_node::getName().c_str());
    ros::spinOnce();

    r.sleep();
    n++;
  }
}
