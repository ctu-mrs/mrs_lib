#include <mrs_lib/SubscribeHandler.h>

using namespace mrs_lib;

template <typename MessageType>
SubscribeHandler_base<MessageType>::SubscribeHandler_base(
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

// --------------------------------------------------------------
// |                    SubscribeHandler_impl                   |
// --------------------------------------------------------------

/* SubscribeHandler_impl implementation //{ */

template <typename MessageType>
SubscribeHandler_impl<MessageType>::SubscribeHandler_impl(
    ros::NodeHandle& nh,
    const std::string& topic_name,
    uint32_t queue_size,
    const ros::TransportHints& transport_hints,
    ros::Duration no_message_timeout,
    const std::string& node_name
    )
  : m_no_message_timeout(no_message_timeout),
    m_node_name(node_name),
    m_got_data(false),
    m_new_data(false),
    m_last_msg_received(ros::Time::now())
{
  try
  {
    m_sub = nh.subscribe(topic_name, queue_size, &SubscribeHandler_impl::data_callback, this, transport_hints);
    m_ok = true;
    const std::string msg = "Subscribed to topic " + topic_name;
    if (m_node_name.empty())
      ROS_INFO_STREAM(msg);
    else
      ROS_INFO_STREAM("[" << m_node_name << "]: " << msg);
  } catch (ros::Exception e)
  {
    const std::string error_msg = "Could not subscribe topic '" + topic_name + "':" + std::string(e.what());
    if (m_node_name.empty())
      ROS_ERROR_STREAM(error_msg);
    else
      ROS_ERROR_STREAM("[" << m_node_name << "]: " << error_msg);
    m_ok = false;
  }
}

template <typename MessageType>
void SubscribeHandler_impl<MessageType>::check_timeout([[maybe_unused]] const ros::TimerEvent& evt)
{
  ros::Duration since_msg = (ros::Time::now() - m_last_msg_received);
  std::lock_guard<std::mutex> lck(m_last_msg_received_mtx);
  if (since_msg > m_no_message_timeout)
  {
    m_ok = false;
    const std::string msg = "Did not receive any message from topic '" + m_sub.getTopic()
                          + "' for " + std::to_string(since_msg.toSec())
                          + "s (" + std::to_string(m_sub.getNumPublishers()) + " publishers on this topic)";

    if (m_node_name.empty())
      ROS_ERROR_STREAM_THROTTLE(1.0, msg);
    else
      ROS_ERROR_STREAM_THROTTLE(1.0, "[" << m_node_name << "]: " << msg);
  }
}

template <typename MessageType>
bool SubscribeHandler_impl<MessageType>::ok()
{
  return m_ok;
}

template <typename MessageType>
bool SubscribeHandler_impl<MessageType>::has_data()
{
  return m_got_data;
}

template <typename MessageType>
bool SubscribeHandler_impl<MessageType>::new_data()
{
  return m_new_data;
}

template <typename MessageType>
bool SubscribeHandler_impl<MessageType>::used_data()
{
  return m_got_data && !m_new_data;
}

template <typename MessageType>
MessageType SubscribeHandler_impl<MessageType>::get_data()
{
  m_new_data = false;
  return m_latest_message;
}

template <typename MessageType>
void SubscribeHandler_impl<MessageType>::data_callback(const MessageType& msg)
{
  /* ROS_ERROR("[SubscribeHandler_impl]: ORIGINAL METHOD CALLED"); */
  m_latest_message = msg;
  m_new_data = true;
  m_got_data = true;
  m_last_msg_received = ros::Time::now();
}

//}


// --------------------------------------------------------------
// |                 SubscribeHandler_threadsafe                |
// --------------------------------------------------------------

/* SubscribeHandler_threadsafe implementation //{ */

template <typename MessageType>
SubscribeHandler_threadsafe<MessageType>::SubscribeHandler_threadsafe(
    ros::NodeHandle& nh,
    const std::string& topic_name,
    uint32_t queue_size,
    const ros::TransportHints& transport_hints,
    ros::Duration no_message_timeout,
    const std::string& node_name
    )
  : SubscribeHandler_impl<MessageType>::SubscribeHandler_impl(nh, topic_name, queue_size, transport_hints, no_message_timeout, node_name)
{
}

template <typename MessageType>
bool SubscribeHandler_threadsafe<MessageType>::ok()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_impl<MessageType>::ok();
}

template <typename MessageType>
bool SubscribeHandler_threadsafe<MessageType>::has_data()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_impl<MessageType>::has_data();
}

template <typename MessageType>
bool SubscribeHandler_threadsafe<MessageType>::new_data()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_impl<MessageType>::new_data();
}

template <typename MessageType>
bool SubscribeHandler_threadsafe<MessageType>::used_data()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_impl<MessageType>::used_data();
}

template <typename MessageType>
MessageType SubscribeHandler_threadsafe<MessageType>::get_data()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_impl<MessageType>::get_data();
}

template <typename MessageType>
void SubscribeHandler_threadsafe<MessageType>::data_callback(const MessageType& msg)
{
  std::lock_guard<std::mutex> lck(m_mtx);
  /* ROS_ERROR("[SubscribeHandler_threadsafe]: OVERRIDE METHOD CALLED"); */
  return SubscribeHandler_impl<MessageType>::data_callback(msg);
}

//}
