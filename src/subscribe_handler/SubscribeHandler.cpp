#include <mrs_lib/SubscribeHandler.h>

namespace mrs_lib
{
  namespace impl
  {

    SubscribeHandler_base::SubscribeHandler_base(
      ros::NodeHandle& nh,
      ros::Duration no_message_timeout,
      const std::string& topic_name,
      const std::string& node_name
      )
      : m_no_message_timeout(no_message_timeout),
        m_topic_name(topic_name),
        m_node_name(node_name),
        m_got_data(false),
        m_new_data(false),
        m_last_msg_received(ros::Time::now())
    {
      if (no_message_timeout != mrs_lib::no_timeout)
        m_timeout_check_timer = nh.createTimer(no_message_timeout, &SubscribeHandler_base::check_timeout, this);
    }
      
    void SubscribeHandler_base::check_timeout([[maybe_unused]] const ros::TimerEvent& evt)
    {
      /* ROS_ERROR("Checking topic %s, delay: %.2f", m_sub.getTopic().c_str(), since_msg.toSec()); */
      std::lock_guard<std::mutex> lck(m_last_msg_received_mtx);
      ros::Duration since_msg = (ros::Time::now() - m_last_msg_received);
      if (since_msg > m_no_message_timeout)
      {
        m_ok = false;
        const std::string msg = "Did not receive any message from topic '" + resolved_topic_name()
                              + "' for " + std::to_string(since_msg.toSec())
                              + "s (" + std::to_string(m_sub.getNumPublishers()) + " publishers on this topic)";
        if (m_node_name.empty())
          ROS_WARN_STREAM(msg);
        else
          ROS_WARN_STREAM("[" << m_node_name << "]: " << msg);
      }
    }

    std::string SubscribeHandler_base::resolved_topic_name()
    {
      std::string ret = m_sub.getTopic();
      if (ret.empty())
        ret = m_topic_name;
      return ret;
    }

    bool SubscribeHandler_base::ok() const
    {
      return m_ok;
    }

    bool SubscribeHandler_base::has_data() const
    {
      return m_got_data;
    }

    bool SubscribeHandler_base::new_data() const
    {
      return m_new_data;
    }

    bool SubscribeHandler_base::used_data() const
    {
      return m_got_data && !m_new_data;
    }

  } // namespace impl
} // namespace mrs_lib

