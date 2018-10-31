#ifndef SUBSCRIBEHANDLER_IMPL_H
#define SUBSCRIBEHANDLER_IMPL_H

#include <mrs_lib/SubscribeHandler.h>

namespace mrs_lib
{
  template <typename MessageType>
  class SubscribeHandler;

  namespace impl
  {
    
    /* SubscribeHandler_base class //{ */
    class SubscribeHandler_base
    {
      public:
        virtual bool ok() const;
        virtual bool has_data() const;
        virtual bool new_data() const;
        virtual bool used_data() const;

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
    //}
    
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
              ros::Duration no_message_timeout = mrs_lib::no_timeout,
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
            const std::string msg = "Subscribed to topic '" + SubscribeHandler_base::m_sub.getTopic() + "'";
            if (SubscribeHandler_base::m_node_name.empty())
              ROS_INFO_STREAM(msg);
            else
              ROS_INFO_STREAM("[" << SubscribeHandler_base::m_node_name << "]: " << msg);
          } catch (ros::Exception e)
          {
            const std::string error_msg = "Could not subscribe topic '" + SubscribeHandler_base::m_sub.getTopic() + "':" + std::string(e.what());
            if (SubscribeHandler_base::m_node_name.empty())
              ROS_ERROR_STREAM(error_msg);
            else
              ROS_ERROR_STREAM("[" << SubscribeHandler_base::m_node_name << "]: " << error_msg);
            SubscribeHandler_base::m_ok = false;
          }
        }

        virtual MessageType get_data()
        {
          SubscribeHandler_base::m_new_data = false;
          return m_latest_message;
        }

      private:
        MessageType m_latest_message;

      protected:
        virtual void data_callback(const MessageType& msg)
        {
          m_latest_message = msg;
          SubscribeHandler_base::m_new_data = true;
          SubscribeHandler_base::m_got_data = true;
          SubscribeHandler_base::m_last_msg_received = ros::Time::now();
        }

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
              ros::Duration no_message_timeout = mrs_lib::no_timeout,
              const std::string& node_name = std::string()
            )
          : SubscribeHandler_impl<MessageType>::SubscribeHandler_impl(nh, topic_name, queue_size, transport_hints, no_message_timeout, node_name)
        {
        }

        virtual bool ok() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return SubscribeHandler_impl<MessageType>::ok();
        }
        virtual bool has_data() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return SubscribeHandler_impl<MessageType>::has_data();
        }
        virtual bool new_data() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return SubscribeHandler_impl<MessageType>::new_data();
        }
        virtual bool used_data() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return SubscribeHandler_impl<MessageType>::used_data();
        }
        virtual MessageType get_data()
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return SubscribeHandler_impl<MessageType>::get_data();
        }

      protected:
        virtual void data_callback(const MessageType& msg)
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          /* ROS_ERROR("[SubscribeHandler_threadsafe]: OVERRIDE METHOD CALLED"); */
          return SubscribeHandler_impl<MessageType>::data_callback(msg);
        }

      private:
        mutable std::mutex m_mtx;
    };
    //}


  } // namespace impl

}

#endif // SUBSCRIBEHANDLER_IMPL_H
