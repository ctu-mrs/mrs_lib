#ifndef SUBSCRIBE_HANDLER_HPP
#define SUBSCRIBE_HANDLER_HPP

#include <mrs_lib/subscribe_handler.h>

namespace mrs_lib
{
  template <typename MessageType>
  class SubscribeHandler;
  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType> >;

  namespace impl
  {
    
    /* SubscribeHandler_base class //{ */
    class SubscribeHandler_base
    {
      public:
        using timeout_callback_t = std::function<void(const std::string&, const ros::Time&, const int)>;

      public:
        virtual bool ok() const;
        virtual bool has_data() const;
        virtual bool new_data() const;
        virtual bool used_data() const;

      protected:
        SubscribeHandler_base(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          const std::string& node_name,
          const ros::Duration& no_message_timeout,
          const timeout_callback_t& timeout_callback
          );

      protected:
        ros::Subscriber m_sub;
    
      protected:
        ros::Duration m_no_message_timeout;
        std::string m_topic_name;
        std::string m_node_name;
    
      protected:
        bool m_ok;
        bool m_got_data;  // whether any data was received
        bool m_new_data;  // whether new data was received since last call to get_data
        bool m_used_data; // whether get_data was successfully called at least once

      protected:
        std::mutex m_last_msg_received_mtx;
        ros::Time m_last_msg_received;
        ros::Timer m_timeout_check_timer;
        timeout_callback_t m_timeout_callback;
        void check_timeout([[maybe_unused]] const ros::TimerEvent& evt);
        void default_timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs);
        std::string resolved_topic_name();

    };
    //}
    
    /* SubscribeHandler_impl class //{ */
    // implements the constructor, get_data() method and data_callback method (non-thread-safe)
    template <typename MessageType, bool time_consistent=false>
    class SubscribeHandler_impl : public SubscribeHandler<MessageType>
    {
      public:
        using timeout_callback_t = SubscribeHandler_base::timeout_callback_t;
        using message_callback_t = std::function<void(SubscribeHandlerPtr<MessageType>)>;

      public:
        SubscribeHandler_impl(
              ros::NodeHandle& nh,
              const std::string& topic_name,
              const std::string& node_name = std::string(),
              const message_callback_t& message_callback = message_callback_t(),
              const ros::Duration& no_message_timeout = mrs_lib::no_timeout,
              const timeout_callback_t& timeout_callback = timeout_callback_t(),
              uint32_t queue_size = 1,
              const ros::TransportHints& transport_hints = ros::TransportHints()
            )
          : SubscribeHandler<MessageType>(
              nh,
              topic_name,
              node_name,
              no_message_timeout,
              timeout_callback
            )
        {
          try
          {
            SubscribeHandler_base::m_sub = nh.subscribe(topic_name, queue_size, &SubscribeHandler_impl::data_callback, this, transport_hints);
            SubscribeHandler_base::m_ok = true;
            const std::string msg = "Subscribed to topic '" + SubscribeHandler_base::m_topic_name + "' -> '" + SubscribeHandler_base::resolved_topic_name() + "'";
            if (SubscribeHandler_base::m_node_name.empty())
              ROS_INFO_STREAM(msg);
            else
              ROS_INFO_STREAM("[" << SubscribeHandler_base::m_node_name << "]: " << msg);
          } catch (ros::Exception e)
          {
            const std::string error_msg = "Could not subscribe topic '" + SubscribeHandler_base::resolved_topic_name() + "': " + std::string(e.what());
            if (SubscribeHandler_base::m_node_name.empty())
              ROS_ERROR_STREAM(error_msg);
            else
              ROS_ERROR_STREAM("[" << SubscribeHandler_base::m_node_name << "]: " << error_msg);
            SubscribeHandler_base::m_ok = false;
          }
          if (!m_message_callback)
          {
            m_message_callback = dummy_message_callback;
          }
        }

        virtual MessageType get_data()
        {
          SubscribeHandler_base::m_new_data = false;
          if (!SubscribeHandler_base::m_got_data)
            ROS_ERROR("[%s]: No data received yet from topic '%s' (forgot to check has_data()?)! Returning empty message.", SubscribeHandler_base::m_node_name.c_str(), SubscribeHandler_base::resolved_topic_name().c_str());
          SubscribeHandler_base::m_used_data = true;
          return m_latest_message;
        }

      private:
        MessageType m_latest_message;
        message_callback_t m_message_callback;

        static void dummy_message_callback(SubscribeHandlerPtr<MessageType> sh_ptr) {}

      protected:
        virtual void data_callback(const MessageType& msg)
        {
          data_callback_impl(msg);
        }

        /* get_header() method //{ */
        template <typename T>
        std_msgs::Header get_header(const T& msg)
        {
          return msg.header;
        }

        template <typename T>
        std_msgs::Header get_header(const boost::shared_ptr<T>& msg)
        {
          return msg->header;
        }
        //}

        // no time consistency check variant
        template<bool check=time_consistent>
        typename std::enable_if<!check>::type data_callback_impl(const MessageType& msg)
        {
          data_callback_unchecked(msg, ros::Time::now());
        }

        // variant with time consistency check
        template<bool check=time_consistent>
        typename std::enable_if<check>::type data_callback_impl(const MessageType& msg)
        {
          ros::Time now = ros::Time::now(); 
          const bool time_reset = check_time_reset(now);
          const bool message_valid = check_message_valid(msg);
          if (message_valid || time_reset)
          {
            if (time_reset)
              ROS_WARN("[%s]: Detected jump back in time of %f. Resetting time consistency checks.", SubscribeHandler_base::m_node_name.c_str(), (SubscribeHandler_base::m_last_msg_received - now).toSec());
            data_callback_unchecked(msg, now);
          } else
          {
            ROS_WARN("[%s]: New message from topic '%s' is older than the latest message, skipping it.", SubscribeHandler_base::m_node_name.c_str(), SubscribeHandler_base::resolved_topic_name().c_str());
          }
        }

        bool check_time_reset(const ros::Time& now)
        {
          return now < SubscribeHandler_base::m_last_msg_received;
        }

        bool check_time_consistent(const MessageType& msg)
        {
          return get_header(msg).stamp >= get_header(m_latest_message).stamp;
        }

        bool check_message_valid(const MessageType& msg)
        {
          return !SubscribeHandler_base::m_got_data || check_time_consistent(msg);
        }

        void data_callback_unchecked(const MessageType& msg, const ros::Time& time)
        {
          m_latest_message = msg;
          SubscribeHandler_base::m_new_data = true;
          SubscribeHandler_base::m_got_data = true;
          SubscribeHandler_base::m_last_msg_received = time;
          SubscribeHandler_base::m_timeout_check_timer.stop();
          SubscribeHandler_base::m_timeout_check_timer.start();
          // FIX THIS SHIT
          m_message_callback(std::shared_ptr<SubscribeHandler<MessageType> >(this));
        }

    };
    //}

    /* SubscribeHandler_threadsafe class //{ */
    template <typename MessageType, bool time_consistent=false>
    class SubscribeHandler_threadsafe : public SubscribeHandler_impl<MessageType, time_consistent>
    {
      private:
        using impl_class_t = impl::SubscribeHandler_impl<MessageType, time_consistent>;

      public:
        using timeout_callback_t = impl::SubscribeHandler_base::timeout_callback_t;
        using message_callback_t = typename impl_class_t::message_callback_t;

      public:
        SubscribeHandler_threadsafe(
              ros::NodeHandle& nh,
              const std::string& topic_name,
              const std::string& node_name = std::string(),
              const message_callback_t& message_callback = message_callback_t(),
              const ros::Duration& no_message_timeout = mrs_lib::no_timeout,
              const timeout_callback_t& timeout_callback = timeout_callback_t(),
              uint32_t queue_size = 1,
              const ros::TransportHints& transport_hints = ros::TransportHints()
            )
          : impl_class_t::SubscribeHandler_impl(nh, topic_name, node_name, message_callback, no_message_timeout, timeout_callback, queue_size, transport_hints)
        {
        }

        virtual bool ok() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::ok();
        }
        virtual bool has_data() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::has_data();
        }
        virtual bool new_data() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::new_data();
        }
        virtual bool used_data() const
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::used_data();
        }
        virtual MessageType get_data()
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::get_data();
        }

      protected:
        virtual void data_callback(const MessageType& msg)
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::data_callback(msg);
        }

      private:
        mutable std::mutex m_mtx;
    };
    //}

  } // namespace impl

}

#endif // SUBSCRIBE_HANDLER_HPP
