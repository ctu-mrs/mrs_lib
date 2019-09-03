#include <mrs_lib/subscribe_handler.h>

namespace mrs_lib
{
  namespace impl
  {

    SubscribeHandler_base::SubscribeHandler_base(
      ros::NodeHandle& nh,
      const std::string& topic_name,
      const std::string& node_name,
      const ros::Duration& no_message_timeout,
      const timeout_callback_t& timeout_callback
      )
      : m_no_message_timeout(no_message_timeout),
        m_topic_name(topic_name),
        m_node_name(node_name),
        m_got_data(false),
        m_new_data(false),
        m_used_data(false),
        m_last_msg_received(ros::Time::now()),
        m_timeout_callback(timeout_callback)
    {
      if (no_message_timeout != mrs_lib::no_timeout)
        m_timeout_check_timer = nh.createTimer(no_message_timeout, &SubscribeHandler_base::check_timeout, this, true /*oneshot*/);
      if (!m_timeout_callback)
        m_timeout_callback = std::bind(&SubscribeHandler_base::default_timeout_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }

    void SubscribeHandler_base::default_timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
    {
      /* ROS_ERROR("Checking topic %s, delay: %.2f", m_sub.getTopic().c_str(), since_msg.toSec()); */
      ros::Duration since_msg = (ros::Time::now() - last_msg);
      const std::string msg = "Did not receive any message from topic '" + topic
                            + "' for " + std::to_string(since_msg.toSec())
                            + "s (" + std::to_string(n_pubs) + " publishers on this topic)";
      if (m_node_name.empty())
        ROS_WARN_STREAM(msg);
      else
        ROS_WARN_STREAM("[" << m_node_name << "]: " << msg);
    }
    
    void SubscribeHandler_base::check_timeout([[maybe_unused]] const ros::TimerEvent& evt)
    {
      ros::Time last_msg;
      {
        std::lock_guard<std::mutex> lck(m_last_msg_received_mtx);
        last_msg = m_last_msg_received;
        m_ok = false;
      }
      const auto n_pubs = m_sub.getNumPublishers();
      m_timeout_check_timer.start();
      m_timeout_callback(resolved_topic_name(), last_msg, n_pubs);
    }

    std::string SubscribeHandler_base::resolved_topic_name()
    {
      std::string ret = m_sub.getTopic();
      if (ret.empty())
        ret = m_topic_name;
      return ret;
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
      return m_used_data;
    }

  } // namespace impl
} // namespace mrs_lib

