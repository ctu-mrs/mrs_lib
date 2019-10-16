/* #include <std_msgs/Int64.h> */
#include <geometry_msgs/PointStamped.h>
#include <mrs_lib/subscribe_handler.h>

/* using message_type = std_msgs::Int64; */
using message_type = geometry_msgs::PointStamped;

void timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
{
  ROS_ERROR_STREAM("Have not received message from topic '" << topic << "' for " << (ros::Time::now()-last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");
}

void message_callback(mrs_lib::SubscribeHandlerPtr<message_type> sh_ptr)
{
  ROS_INFO_STREAM("RECEIVED '" << *(sh_ptr->get_data()) << "'");
}

mrs_lib::SubscribeHandlerPtr<message_type> sh_ptr;
void thread_fcn([[maybe_unused]] const ros::TimerEvent& evt)
{
  if (sh_ptr->has_data())
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "time of last message: " << sh_ptr->last_message_time());
  }
}

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("subscribe_handler_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  /* name of the topic to be handled */
  const std::string topic_name = "test_topic";
  /* after this duration without receiving messages on the handled topic, the timeout_callback will be called */
  const ros::Duration no_message_timeout = ros::Duration(2.0);
  /* whether mutexes should be used to prevent data races (set to true in a multithreaded scenario such as nodelets) */
  const bool threadsafe = true;
  const bool autostart = false;
  const bool time_consistent = true;
  const uint32_t queue_size = 5;
  const ros::TransportHints& transport_hints = ros::TransportHints();

  ROS_INFO("[%s]: Creating SubscribeHandlers using SubscribeMgr.", node_name.c_str());
  mrs_lib::SubscribeMgr smgr(nh, node_name);

  /* This is how a new SubscribeHandler object is initialized. */ 
  sh_ptr = smgr.create_handler<message_type, time_consistent>(
            topic_name,
            no_message_timeout,
            timeout_callback,
            message_callback,
            threadsafe,
            autostart,
            queue_size,
            transport_hints
            );

  std::vector<ros::Timer> timers;
  for (int it = 0; it < 200; it++)
  {
    timers.push_back(nh.createTimer(ros::Duration(1e-3), thread_fcn));
  }

  /* Type of the message may be accessed by C++11 decltype in case of need */ 
  /* using message_type = std::remove_const<decltype(handler1)::element_type::message_type>::type; */
  ros::Publisher pub = nh.advertise<message_type>(topic_name, 5);

  /* Now let's just spin to process calbacks until the user decides to stop the program. */ 
  ros::Rate r(10);
  int n = 0;
  message_type msg;
  while (ros::ok() && n < 50)
  {
    if (n % 10 == 0)
    {
      ROS_INFO("[%s]: STARTING handler", ros::this_node::getName().c_str());
      sh_ptr->start();
    } else if (n % 5 == 0)
    {
      ROS_INFO("[%s]: STOPPING handler", ros::this_node::getName().c_str());
      sh_ptr->stop();
    }

    if (n % 4 == 0)
      msg.header.stamp = ros::Time::now() - ros::Duration(10.0);
    else
      msg.header.stamp = ros::Time::now();
    msg.point.x = msg.point.y = msg.point.z = n;
    /* msg.data = n; */
    ROS_INFO_STREAM("PUBLISHING message '" << msg << "'");
    pub.publish(msg);

    ROS_INFO("[%s]: Spinning", ros::this_node::getName().c_str());
    ros::spinOnce();

    r.sleep();
    n++;
  }
}
