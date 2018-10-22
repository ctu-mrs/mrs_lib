#include <mrs_lib/SubscribeHandler.h>

using namespace mrs_lib;
using namespace mrs_lib::impl;

// --------------------------------------------------------------
// |                    SubscribeHandler_impl                   |
// --------------------------------------------------------------

/* SubscribeHandler_impl implementation //{ */

template <typename MessageType>
MessageType SubscribeHandler_impl<MessageType>::get_data()
{
  SubscribeHandler_base::m_new_data = false;
  return m_latest_message;
}

template <typename MessageType>
void SubscribeHandler_impl<MessageType>::data_callback(const MessageType& msg)
{
  /* ROS_ERROR("[SubscribeHandler_impl]: ORIGINAL METHOD CALLED"); */
  m_latest_message = msg;
  SubscribeHandler_base::m_new_data = true;
  SubscribeHandler_base::m_got_data = true;
  SubscribeHandler_base::m_last_msg_received = ros::Time::now();
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
