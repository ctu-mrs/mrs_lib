#ifndef SUBRSCRIBEHANDLER_H
#define SUBRSCRIBEHANDLER_H

#include <ros/ros.h>
#include <string>
#include <mutex>

namespace mrs_lib
{

  static const ros::Duration no_timeout = ros::Duration(0);

  class SubscribeHandler_base
  {
    public:
      SubscribeHandler_base();
      virtual bool ok() = 0;
      virtual bool has_data() = 0;
      virtual bool new_data() = 0;
      virtual bool used_data() = 0;

    protected:
      SubscribeHandler_base(
        ros::NodeHandle& nh,
        ros::Duration no_message_timeout,
        const std::string& topic_name,
        const std::string& node_name
        );

    protected:
      ros::Subscriber m_sub;
  
    protected:
      bool m_ok;
      bool m_got_data; // whether any data was received
      bool m_new_data; // whether new data was received since last call to get_data

    protected:
      ros::Duration m_no_message_timeout;
      std::string m_topic_name;
      std::string m_node_name;
  
    protected:
      std::mutex m_last_msg_received_mtx;
      ros::Time m_last_msg_received;
      ros::Timer m_timeout_check_timer;
      void check_timeout([[maybe_unused]] const ros::TimerEvent& evt);

  };

  /* SubscribeHandler class //{ */
  // adds the get_data templated method - this is the class the user should use
  // for pointers etc.
  template <typename MessageType>
  class SubscribeHandler : virtual public SubscribeHandler_base
  {
    public:
      virtual MessageType get_data() = 0;

    protected:
      SubscribeHandler() {};
  };
  //}

  namespace impl
  {

  /* SubscribeHandler_impl class //{ */
  // implements the constructor, get_data() method and data_callback method (non-thread-safe)
  template <typename MessageType>
  class SubscribeHandler_impl : public SubscribeHandler<MessageType>
  {
    public:
      SubscribeHandler_impl(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          );

      virtual bool ok();
      virtual bool has_data();
      virtual bool new_data();
      virtual bool used_data();
      virtual MessageType get_data();

    private:
      void check_timeout([[maybe_unused]] const ros::TimerEvent& evt);
      MessageType m_latest_message;

    protected:
      virtual void data_callback(const MessageType& msg);

  };
  //}

  /* SubscribeHandler_threadsafe class //{ */
  template <typename MessageType>
  class SubscribeHandler_threadsafe : public SubscribeHandler_impl<MessageType>
  {
    public:
      SubscribeHandler_threadsafe(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler_impl<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          );

      virtual bool ok();
      virtual bool has_data();
      virtual bool new_data();
      virtual bool used_data();
      virtual MessageType get_data();

    protected:
      virtual void data_callback(const MessageType& msg);

    private:
      std::mutex m_mtx;
  };
  //}

  } // namespace impl

  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType>>;

  /* SubscriberMgr class //{ */
  class SubscribeMgr
  {
    public:
      SubscribeMgr() : m_load_successful(true) {};

      template <typename MessageType>
      SubscribeHandlerPtr<MessageType> create_handler(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          )
      {
        SubscribeHandlerPtr<MessageType> ptr = std::make_shared<impl::SubscribeHandler_impl<MessageType> >
          (
            nh,
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
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          )
      {
        SubscribeHandlerPtr<MessageType> ptr = std::make_shared<impl::SubscribeHandler_threadsafe<MessageType> >
          (
            nh,
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
      bool m_load_successful;
  
  };
  //}

}

#include "SubscribeHandler.tpp"

#endif // SUBRSCRIBEHANDLER_H
