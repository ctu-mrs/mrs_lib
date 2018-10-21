#include <mrs_lib/SubscribeHandler.h>

using namespace mrs_lib;

template <typename MessageType>
const ros::Duration SubscribeHandler<MessageType>::no_timeout = ros::Duration(0);

// --------------------------------------------------------------
// |                    SubscribeHandler_base                   |
// --------------------------------------------------------------

/* SubscribeHandler_base implementation //{ */

template <typename MessageType>
SubscribeHandler_base<MessageType>::SubscribeHandler_base(
    ros::NodeHandle& nh,
    const std::string& topic_name,
    uint32_t queue_size,
    const ros::TransportHints& transport_hints,
    ros::Duration no_message_timeout,
    const std::string& node_name
    )
  : m_no_message_timeout(no_message_timeout),
    m_node_name(node_name),
    m_new_data_ready(false),
    m_last_msg_received(ros::Time::now())
{
  try
  {
    nh.subscribe(topic_name, queue_size, &SubscribeHandler_base::data_callback, this, transport_hints);
    m_ok = true;
  } catch (ros::Exception e)
  {
    const std::string error_msg = "Could not subscribe topic '" + topic_name + "':" + std::string(e.what());
    if (m_node_name.empty())
      ROS_ERROR_STREAM(error_msg);
    else
      ROS_ERROR_STREAM("[" << m_node_name << "]" << error_msg);
    m_ok = false;
  }
}

template <typename MessageType>
bool SubscribeHandler_base<MessageType>::ok()
{
  return m_ok;
}

template <typename MessageType>
bool SubscribeHandler_base<MessageType>::has_data()
{
  return m_new_data_ready;
}

template <typename MessageType>
MessageType SubscribeHandler_base<MessageType>::get_data(bool reset_data_flag)
{
  if (reset_data_flag)
    m_new_data_ready = false;
  return m_latest_message;
}

template <typename MessageType>
MessageType SubscribeHandler_base<MessageType>::get_data_reset()
{
  return get_data(true);
}

template <typename MessageType>
void SubscribeHandler_base<MessageType>::data_callback(const MessageType& msg)
{
  ROS_ERROR("[SubscribeHandler_base]: ORIGINAL METHOD CALLED");
  m_latest_message = msg;
  m_new_data_ready = true;
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
  : SubscribeHandler_base<MessageType>::SubscribeHandler_base(nh, topic_name, queue_size, transport_hints, no_message_timeout, node_name)
{
}

template <typename MessageType>
bool SubscribeHandler_threadsafe<MessageType>::ok()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_base<MessageType>::ok();
}

template <typename MessageType>
bool SubscribeHandler_threadsafe<MessageType>::has_data()
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_base<MessageType>::has_data();
}

template <typename MessageType>
MessageType SubscribeHandler_threadsafe<MessageType>::get_data(bool reset_data_flag)
{
  std::lock_guard<std::mutex> lck(m_mtx);
  return SubscribeHandler_base<MessageType>::get_data(reset_data_flag);
}

template <typename MessageType>
void SubscribeHandler_threadsafe<MessageType>::data_callback(const MessageType& msg)
{
  std::lock_guard<std::mutex> lck(m_mtx);
  ROS_ERROR("[SubscribeHandler_threadsafe]: OVERRIDE METHOD CALLED");
  return SubscribeHandler_base<MessageType>::data_callback(msg);
}

//}
