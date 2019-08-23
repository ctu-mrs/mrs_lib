#include <std_msgs/Bool.h>
#include <mrs_lib/subscribe_handler.h>

using timeout_callback_t = mrs_lib::impl::SubscribeHandler_base::timeout_callback_t;
void timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
{
  ROS_ERROR_STREAM("Have not received message from topic '" << topic << "' for " << (ros::Time::now()-last_msg).toSec() << " seconds (" << n_pubs << " publishers on topic)");
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "subscribe_handler_tests");
  ros::NodeHandle nh;

  const std::string topic_name = "test_topic";
  const ros::Duration no_message_timeout = ros::Duration(5.0);
  const timeout_callback_t timeout_callback_f = timeout_callback;
  const bool threadsafe = false;
  const uint32_t queue_size = 5;
  const ros::TransportHints& transport_hints = ros::TransportHints().tcpNoDelay();

  mrs_lib::SubscribeMgr smgr(nh);

  auto handler1 = smgr.create_handler<std_msgs::BoolConstPtr>(
            topic_name,
            no_message_timeout,
            timeout_callback_f,
            threadsafe,
            queue_size,
            transport_hints
            );

  ros::spin();
}

