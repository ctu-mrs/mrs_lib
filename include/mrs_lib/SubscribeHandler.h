#ifndef SUBRSCRIBEHANDLER_H
#define SUBRSCRIBEHANDLER_H

#include <ros/ros.h>
#include <string>
#include <mutex>

namespace mrs_lib
{

  static const ros::Duration no_timeout = ros::Duration(0);

  namespace impl
  {
    class SubscribeHandler_base
    {
      public:
        SubscribeHandler_base() {
    ROS_ERROR("[%s]: INCORRECT constructor", ros::this_node::getName().c_str());
        };
        virtual bool ok();
        virtual bool has_data();
        virtual bool new_data();
        virtual bool used_data();

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
        ros::Duration m_no_message_timeout;
        std::string m_topic_name;
        std::string m_node_name;
    
      protected:
        bool m_ok;
        bool m_got_data; // whether any data was received
        bool m_new_data; // whether new data was received since last call to get_data

      protected:
        std::mutex m_last_msg_received_mtx;
        ros::Time m_last_msg_received;
        ros::Timer m_timeout_check_timer;
        void check_timeout([[maybe_unused]] const ros::TimerEvent& evt);

    };
  }

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
        : SubscribeHandler_base(
            nh,
            no_message_timeout,
            topic_name,
            node_name
          )
      {};
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
            )
          : SubscribeHandler<MessageType>(
              nh,
              no_message_timeout,
              topic_name,
              node_name
            )
        {
          try
          {
            SubscribeHandler_base::m_sub = nh.subscribe(topic_name, queue_size, &SubscribeHandler_impl::data_callback, this, transport_hints);
            SubscribeHandler_base::m_ok = true;
            const std::string msg = "Subscribed to topic '" + topic_name + "'";
            if (SubscribeHandler_base::m_node_name.empty())
              ROS_INFO_STREAM(msg);
            else
              ROS_INFO_STREAM("[" << SubscribeHandler_base::m_node_name << "]: " << msg);
          } catch (ros::Exception e)
          {
            const std::string error_msg = "Could not subscribe topic '" + topic_name + "':" + std::string(e.what());
            if (SubscribeHandler_base::m_node_name.empty())
              ROS_ERROR_STREAM(error_msg);
            else
              ROS_ERROR_STREAM("[" << SubscribeHandler_base::m_node_name << "]: " << error_msg);
            SubscribeHandler_base::m_ok = false;
          }
        }

        virtual MessageType get_data();

      private:
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
