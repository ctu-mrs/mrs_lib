#ifndef SUBRSCRIBEHANDLER_H
#define SUBRSCRIBEHANDLER_H

#include <ros/ros.h>
#include <string>
#include <mutex>
#include <impl/SubscribeHandler_impl.h>

namespace mrs_lib
{

  static const ros::Duration no_timeout = ros::Duration(0);

  /* SubscribeHandler class //{ */
  // adds the get_data templated method - this is the class the user should use
  // for pointers etc.
  template <typename MessageType>
  class SubscribeHandler : public impl::SubscribeHandler_base
  {
    public:
      virtual MessageType get_data() = 0;

    protected:
      SubscribeHandler(
        ros::NodeHandle& nh,
        ros::Duration no_message_timeout,
        const std::string& topic_name,
        const std::string& node_name
        )
        : impl::SubscribeHandler_base(
            nh,
            no_message_timeout,
            topic_name,
            node_name
          )
      {};
  };
  //}

  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType>>;

  /* SubscriberMgr class //{ */
  class SubscribeMgr
  {
    public:
      SubscribeMgr(ros::NodeHandle& nh) : m_nh(nh), m_load_successful(true) {};

      template <typename MessageType>
      SubscribeHandlerPtr<MessageType> create_handler(
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          )
      {
        SubscribeHandlerPtr<MessageType> ptr = std::make_shared<impl::SubscribeHandler_impl<MessageType> >
          (
            m_nh,
            topic_name,
            queue_size,
            transport_hints,
            no_message_timeout,
            node_name
          );
        m_load_successful = m_load_successful && ptr->ok();
        return ptr;
      }
  
      template <typename MessageType>
      SubscribeHandlerPtr<MessageType> create_handler_threadsafe(
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          )
      {
        SubscribeHandlerPtr<MessageType> ptr = std::make_shared<impl::SubscribeHandler_threadsafe<MessageType> >
          (
            m_nh,
            topic_name,
            queue_size,
            transport_hints,
            no_message_timeout,
            node_name
          );
        m_load_successful = m_load_successful && ptr->ok();
        return ptr;
      }

      bool loaded_successfully()
      {
        return m_load_successful;
      }

    private:
      ros::NodeHandle m_nh;
      bool m_load_successful;
  
  };
  //}

}

#endif // SUBRSCRIBEHANDLER_H
