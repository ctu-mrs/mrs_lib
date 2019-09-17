#ifndef SUBSCRIBE_HANDLER_HPP
#define SUBSCRIBE_HANDLER_HPP

#include <mrs_lib/subscribe_handler.h>

namespace mrs_lib
{
  namespace impl
  {
    
    /* SubscribeHandler_impl class //{ */
    // implements the constructor, get_data() method and data_callback method (non-thread-safe)
    template <typename MessageType>
    class SubscribeHandler_impl
    {
      public:
        using timeout_callback_t = typename SubscribeHandler<MessageType>::timeout_callback_t;
        using message_callback_t = typename SubscribeHandler<MessageType>::message_callback_t;

      private:
        friend class SubscribeMgr;
        friend class SubscribeHandler<MessageType>;

      public:
        SubscribeHandler_impl(
              ros::NodeHandle& nh,
              const std::string& topic_name,
              const std::string& node_name = std::string(),
              const message_callback_t& message_callback = message_callback_t(),
              const ros::Duration& no_message_timeout = mrs_lib::no_timeout,
              const timeout_callback_t& timeout_callback = timeout_callback_t(),
              const bool time_consistent = false,
              const uint32_t queue_size = 10,
              const ros::TransportHints& transport_hints = ros::TransportHints()
            )
          : m_nh(nh),
            m_no_message_timeout(no_message_timeout),
            m_topic_name(topic_name),
            m_node_name(node_name),
            m_got_data(false),
            m_new_data(false),
            m_used_data(false),
            m_last_msg_received(ros::Time::now()),
            m_timeout_callback(timeout_callback),
            m_message_callback(message_callback),
            m_time_consistent(time_consistent),
            m_queue_size(queue_size),
            m_transport_hints(transport_hints)
        {
          if (!m_message_callback)
            m_message_callback = dummy_message_callback;

          if (!m_timeout_callback)
            m_timeout_callback = std::bind(&SubscribeHandler_impl<MessageType>::default_timeout_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

          if (no_message_timeout != mrs_lib::no_timeout)
            m_timeout_check_timer = m_nh.createTimer(no_message_timeout, &SubscribeHandler_impl<MessageType>::check_timeout, this, true /*oneshot*/, false /*autostart*/);

          const std::string msg = "Subscribed to topic '" + m_topic_name + "' -> '" + resolved_topic_name() + "'";
          if (m_node_name.empty())
            ROS_INFO_STREAM(msg);
          else
            ROS_INFO_STREAM("[" << m_node_name << "]: " << msg);
        }

      protected:
        virtual MessageType get_data()
        {
          m_new_data = false;
          assert(m_got_data);
          if (!m_got_data)
            ROS_ERROR("[%s]: No data received yet from topic '%s' (forgot to check has_data()?)! Returning empty message.", m_node_name.c_str(), resolved_topic_name().c_str());
          m_used_data = true;
          return m_latest_message;
        }

        virtual bool has_data() const
        {
          return m_got_data;
        }

        virtual bool new_data() const
        {
          return m_new_data;
        }

        virtual bool used_data() const
        {
          return m_used_data;
        }

        virtual void start()
        {
          m_timeout_check_timer.start();
          if (m_time_consistent)
            m_sub = m_nh.subscribe(m_topic_name, m_queue_size, &SubscribeHandler_impl<MessageType>::data_callback_time_consistent, this, m_transport_hints);
          else
            m_sub = m_nh.subscribe(m_topic_name, m_queue_size, &SubscribeHandler_impl<MessageType>::data_callback, this, m_transport_hints);
        }

        virtual void stop()
        {
          m_timeout_check_timer.stop();
          m_sub.shutdown();
        }

      public:
        void set_owner_ptr(const SubscribeHandlerPtr<MessageType>& ptr)
        {
          m_ptr = ptr;
        }

      protected:
        ros::NodeHandle m_nh;
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

      private:
        std::weak_ptr<SubscribeHandler<MessageType>> m_ptr;
        MessageType m_latest_message;
        message_callback_t m_message_callback;
        bool m_time_consistent;

      private:
        uint32_t m_queue_size;
        ros::TransportHints m_transport_hints;

        static void dummy_message_callback([[maybe_unused]] SubscribeHandlerPtr<MessageType> sh_ptr) {};

      protected:
        void default_timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
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
        
        void check_timeout([[maybe_unused]] const ros::TimerEvent& evt)
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

        std::string resolved_topic_name()
        {
          std::string ret = m_sub.getTopic();
          if (ret.empty())
            ret = m_topic_name;
          return ret;
        }

        void data_callback(const MessageType& msg)
        {
          data_callback_unchecked(msg, ros::Time::now());
        }

        void data_callback_time_consistent(const MessageType& msg)
        {
          ros::Time now = ros::Time::now(); 
          const bool time_reset = check_time_reset(now);
          const bool message_valid = check_message_valid(msg);
          if (message_valid || time_reset)
          {
            if (time_reset)
              ROS_WARN("[%s]: Detected jump back in time of %f. Resetting time consistency checks.", m_node_name.c_str(), (m_last_msg_received - now).toSec());
            data_callback_unchecked(msg, now);
          } else
          {
            ROS_WARN("[%s]: New message from topic '%s' is older than the latest message, skipping it.", m_node_name.c_str(), resolved_topic_name().c_str());
          }
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

        template <typename T>
        class HasToString
        {
        private:
            typedef char YesType[1];
            typedef char NoType[2];

            template <typename C> static YesType& test( decltype(&C::ToString) ) ;
            template <typename C> static NoType& test(...);


        public:
            enum { value = sizeof(test<T>(0)) == sizeof(YesType) };
        };
        template <typename T>
        typename std::enable_if<has_header<T>::value, std_msgs::Header>::type
        get_header(const T& msg)
        {
          assert(false);
          ROS_ERROR("[%s]: SubscribeHandler: Cannot call get_header() for a message without a header! Cannot enforce time consistency for message type '%s'", m_node_name.c_str(), typeid(T).name());
          return std_msgs::Header();
        }
        //}

        bool check_time_reset(const ros::Time& now)
        {
          return now < m_last_msg_received;
        }

        bool check_time_consistent(const MessageType& msg)
        {
          return get_header(msg).stamp >= get_header(m_latest_message).stamp;
        }

        bool check_message_valid(const MessageType& msg)
        {
          return !m_got_data || check_time_consistent(msg);
        }

        virtual void process_new_message(const MessageType& msg, const ros::Time& time)
        {
          m_timeout_check_timer.stop();
          m_latest_message = msg;
          m_new_data = true;
          m_got_data = true;
          m_last_msg_received = time;
          m_timeout_check_timer.start();
        }

        void data_callback_unchecked(const MessageType& msg, const ros::Time& time)
        {
          process_new_message(msg, time);
          m_message_callback(std::shared_ptr(m_ptr));
        }
    };
    //}

    /* SubscribeHandler_threadsafe class //{ */
    template <typename MessageType>
    class SubscribeHandler_threadsafe : public SubscribeHandler_impl<MessageType>
    {
      private:
        using impl_class_t = impl::SubscribeHandler_impl<MessageType>;

      public:
        using timeout_callback_t = typename impl_class_t::timeout_callback_t;
        using message_callback_t = typename impl_class_t::message_callback_t;

        friend class SubscribeMgr;
        friend class SubscribeHandler<MessageType>;

      public:
        SubscribeHandler_threadsafe(
              ros::NodeHandle& nh,
              const std::string& topic_name,
              const std::string& node_name = std::string(),
              const message_callback_t& message_callback = message_callback_t(),
              const ros::Duration& no_message_timeout = mrs_lib::no_timeout,
              const timeout_callback_t& timeout_callback = timeout_callback_t(),
              const bool time_consistent = false,
              const uint32_t queue_size = 10,
              const ros::TransportHints& transport_hints = ros::TransportHints()
            )
          : impl_class_t::SubscribeHandler_impl(nh, topic_name, node_name, message_callback, no_message_timeout, timeout_callback, time_consistent, queue_size, transport_hints)
        {
        }

      protected:
        virtual bool has_data() const override
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::has_data();
        }
        virtual bool new_data() const override
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::new_data();
        }
        virtual bool used_data() const override
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::used_data();
        }
        virtual MessageType get_data() override
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::get_data();
        }
        virtual void start() override
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::start();
        }
        virtual void stop() override
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::stop();
        }

      protected:
        virtual void process_new_message(const MessageType& msg, const ros::Time& time) override
        {
          std::lock_guard<std::mutex> lck(m_mtx);
          return impl_class_t::process_new_message(msg, time);
        }

      private:
        mutable std::mutex m_mtx;
    };
    //}

  } // namespace impl

}

#endif // SUBSCRIBE_HANDLER_HPP
