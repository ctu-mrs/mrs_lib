/* #include <std_msgs/Int64.h> */
#include <geometry_msgs/PointStamped.h>
#include <mrs_lib/subscribe_handler.h>

/* using message_type = std_msgs::Int64; */
using message_type = geometry_msgs::PointStamped;

void timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
{
  ROS_ERROR_STREAM("Have not received message from topic '" << topic << "' for " << (ros::Time::now()-last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");
}

bool started = true;
void message_callback(mrs_lib::SubscribeHandler<message_type>& sh)
{
  if (!started)
    ROS_ERROR("NOT STARTED, SHOULDN'T HAVE RECEIVED!");
  ROS_INFO_STREAM("RECEIVED '" << sh.getMsg() << "'");
}

mrs_lib::SubscribeHandler<message_type> sh;
void thread_fcn([[maybe_unused]] const ros::TimerEvent& evt)
{
  if (sh.hasMsg())
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "time of last message: " << sh.lastMsgTime());
  }
}

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("subscribe_handler_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.autostart = false;
  /* after this duration without receiving messages on the handled topic, the timeout_callback will be called */
  shopts.no_message_timeout = ros::Duration(2.0);
  /* whether mutexes should be used to prevent data races (set to true in a multithreaded scenario such as nodelets) */
  shopts.threadsafe = false;
  shopts.autostart = false;
  shopts.queue_size = 5;
  shopts.transport_hints = ros::TransportHints();

  ROS_INFO("[%s]: Creating SubscribeHandlers.", node_name.c_str());

  /* name of the topic to be handled */
  const std::string topic_name = "/test_topic/pt";
  /* This is how a new SubscribeHandler object is initialized. */ 
  mrs_lib::construct_object(
      sh,
      shopts,
      topic_name,
      timeout_callback,
      message_callback
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
      started = true;
      sh.start();
    } else if (n % 5 == 0)
    {
      ROS_INFO("[%s]: STOPPING handler", ros::this_node::getName().c_str());
      started = false;
      sh.stop();
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
