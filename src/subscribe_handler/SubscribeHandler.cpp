#include "SubscribeHandler.h"

SubscribeHandler_base::SubscribeHandler_base(
  ros::NodeHandle& nh,
  ros::Duration no_message_timeout,
  const std::string& topic_name,
  const std::string& node_name
  )
  : m_no_message_timeout(no_message_timeout),
    m_node_name(node_name),
    m_got_data(false),
    m_new_data(false),
    m_last_msg_received(ros::Time::now())
{


  if (no_message_timeout != mrs_lib::no_timeout)
    m_timeout_check_timer = nh.createTimer(no_message_timeout, &SubscribeHandler_impl::check_timeout, this);
}
