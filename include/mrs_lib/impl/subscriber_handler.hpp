// clang: MatousFormat

#ifndef SUBSCRIBER_HANDLER_HPP
#define SUBSCRIBER_HANDLER_HPP

#ifndef SUBSCRIBER_HANDLER_H
#include <mrs_lib/subscriber_handler.h>
#endif

#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mutex>
#include <condition_variable>

namespace mrs_lib
{
  /* SubscriberHandler::Impl class //{ */
  // implements the constructor, getMsg() method and data_callback method (non-thread-safe)
  template <typename MessageType>
  class SubscriberHandler<MessageType>::Impl
  {
  public:
    using timeout_callback_t = typename SubscriberHandler<MessageType>::timeout_callback_t;
    using message_callback_t = typename SubscriberHandler<MessageType>::message_callback_t;
    using data_callback_t = std::function<void(const typename MessageType::ConstSharedPtr&)>;

  private:
    friend class SubscriberHandler<MessageType>;

  public:
    /* constructor //{ */
    Impl(const SubscriberHandlerOptions& options, const message_callback_t& message_callback = message_callback_t())
        : m_node(options.node),
          m_qos(options.qos),
          m_sub_opts(options.subscription_options),
          m_topic_name(options.topic_name),
          m_node_name(options.node_name),
          m_got_data(false),
          m_new_data(false),
          m_used_data(false),
          m_timeout_manager(options.timeout_manager),
          m_latest_message_time(0),
          m_latest_message(nullptr),
          m_message_callback(message_callback)
    {

      // initialize the callback for the TimeoutManager
      if (options.timeout_callback)
      {
        m_timeout_mgr_callback = std::bind(options.timeout_callback, m_topic_name, std::placeholders::_1);
      } else
      {
        m_timeout_mgr_callback = std::bind(&Impl::default_timeout_callback, this, m_topic_name, std::placeholders::_1);
      }

      if (options.no_message_timeout != mrs_lib::no_timeout)
      {

        // initialize a new TimeoutManager if not provided by the user
        if (!m_timeout_manager)
        {
          m_timeout_manager =
              std::make_shared<mrs_lib::TimeoutManager>(m_node, rclcpp::Rate(1.0 / (options.no_message_timeout.seconds() * 0.5), m_node->get_clock()));
        }

        // register the timeout callback with the TimeoutManager
        m_timeout_id = m_timeout_manager->registerNew(options.no_message_timeout, m_timeout_mgr_callback, m_node->get_clock()->now());
      }

      const std::string msg = "Subscribed to topic '" + m_topic_name + "' -> '" + m_topic_name + "'";

      RCLCPP_INFO_STREAM(m_node->get_logger(), msg);
    }
    //}

    virtual ~Impl() = default;

  public:
    /* getMsg() method //{ */
    virtual typename MessageType::ConstSharedPtr getMsg()
    {
      m_new_data = false;
      m_used_data = true;
      return peekMsg();
    }
    //}

    /* peekMsg() method //{ */
    virtual typename MessageType::ConstSharedPtr peekMsg() const
    {
      /* assert(m_got_data); */
      /* if (!m_got_data) */
      /*   ROS_ERROR("[%s]: No data received yet from topic '%s' (forgot to check hasMsg()?)! Returning empty message.", m_node_name.c_str(), */
      /*             topicName().c_str()); */
      return m_latest_message;
    }
    //}

    /* hasMsg() method //{ */
    virtual bool hasMsg() const
    {
      return m_got_data;
    }
    //}

    /* newMsg() method //{ */
    virtual bool newMsg() const
    {
      return m_new_data;
    }
    //}

    /* usedMsg() method //{ */
    virtual bool usedMsg() const
    {
      return m_used_data;
    }
    //}

    /* waitForNew() method //{ */
    virtual typename MessageType::ConstSharedPtr waitForNew(const rclcpp::Duration& timeout)
    {
      // convert the ros type to chrono type
      const std::chrono::duration<float> chrono_timeout(timeout.seconds());
      // lock the mutex guarding the m_new_data flag
      std::unique_lock lock(m_new_data_mtx);
      // if new data is available, return immediately, otherwise attempt to wait for new data using the respective condition variable
      if (m_new_data)
        return getMsg();
      else if (m_new_data_cv.wait_for(lock, chrono_timeout) == std::cv_status::no_timeout && m_new_data)
        return getMsg();
      else
        return nullptr;
    };
    //}

    /* lastMsgTime() method //{ */
    virtual rclcpp::Time lastMsgTime() const
    {
      return m_latest_message_time;
    };
    //}

    /* topicName() method //{ */
    virtual std::string topicName() const
    {
      if (m_sub == nullptr)
      {
        return "";
      }

      std::string ret = m_sub->get_topic_name();

      if (ret.empty())
      {
        ret = m_node->get_node_topics_interface()->resolve_topic_name(ret);
      }

      return ret;
    }
    //}

    /* getNumPublishers() method //{ */
    virtual uint32_t getNumPublishers() const
    {
      return m_sub->get_publisher_count();
    };
    //}

    /* setNoMessageTimeout() method //{ */
    virtual void setNoMessageTimeout(const rclcpp::Duration& timeout)
    {
      if (timeout == mrs_lib::no_timeout)
      {
        // if there is a timeout callback already registered but the user wants to disable it, pause it
        if (m_timeout_manager != nullptr && m_timeout_id.has_value())
          m_timeout_manager->pause(m_timeout_id.value());
        // otherwise, there is no callback, so nothing to do
      } else
      {
        // if there is no callback manager, create it
        if (m_timeout_manager == nullptr)
          m_timeout_manager = std::make_shared<mrs_lib::TimeoutManager>(m_node, rclcpp::Rate(1.0 / (timeout.seconds() * 0.5), m_node->get_clock()));

        // if there is an existing timeout callback registered, change its timeout
        if (m_timeout_id.has_value())
          m_timeout_manager->change(m_timeout_id.value(), timeout, m_timeout_mgr_callback, m_node->get_clock()->now());

        // otherwise, register it
        else
          m_timeout_id = m_timeout_manager->registerNew(timeout, m_timeout_mgr_callback, m_node->get_clock()->now());
      }
    }
    //}

    /* start() method //{ */
    virtual void start()
    {
      if (m_timeout_manager && m_timeout_id.has_value())
        m_timeout_manager->start(m_timeout_id.value(), m_node->get_clock()->now());

      const std::function<void(const typename MessageType::SharedPtr)> cbk = std::bind(&Impl::data_callback, this, std::placeholders::_1);

      m_sub = m_node->create_subscription<MessageType>(m_topic_name, m_qos, cbk, m_sub_opts);
    }
    //}

    /* stop() method //{ */
    virtual void stop()
    {
      if (m_timeout_manager && m_timeout_id.has_value())
      {
        m_timeout_manager->pause(m_timeout_id.value());
      }

      m_sub.reset();
    }
    //}

  protected:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<MessageType>::SharedPtr m_sub;
    rclcpp::QoS m_qos = rclcpp::SystemDefaultsQoS();
    rclcpp::SubscriptionOptions m_sub_opts;

  protected:
    std::string m_topic_name;
    std::string m_node_name;

  protected:
    bool m_got_data;  // whether any data was received

    mutable std::mutex m_new_data_mtx;
    mutable std::condition_variable m_new_data_cv;
    bool m_new_data;  // whether new data was received since last call to get_data

    bool m_used_data;  // whether get_data was successfully called at least once

  protected:
    std::shared_ptr<mrs_lib::TimeoutManager> m_timeout_manager;
    std::optional<mrs_lib::TimeoutManager::timeout_id_t> m_timeout_id;
    mrs_lib::TimeoutManager::callback_t m_timeout_mgr_callback;

  protected:
    rclcpp::Time m_latest_message_time;
    typename MessageType::ConstSharedPtr m_latest_message;
    message_callback_t m_message_callback;

  protected:
    /* default_timeout_callback() method //{ */
    void default_timeout_callback(const std::string& topic_name, const rclcpp::Time& last_msg)
    {
      const rclcpp::Duration since_msg = (m_node->get_clock()->now() - last_msg);
      const auto n_pubs = m_sub->get_publisher_count();
      const std::string txt = "Did not receive any message from topic '" + topic_name + "' for " + std::to_string(since_msg.seconds()) + "s ("
                              + std::to_string(n_pubs) + " publishers on this topic)";

      RCLCPP_WARN_STREAM(m_node->get_logger(), txt);
    }
    //}

    /* process_new_message() method //{ */
    void process_new_message(const typename MessageType::ConstSharedPtr& msg)
    {
      m_latest_message_time = m_node->get_clock()->now();
      m_latest_message = msg;
      // If the message callback is registered, the new data will immediately be processed,
      // so reset the flag. Otherwise, set the flag.
      m_new_data = !m_message_callback;
      m_got_data = true;
      m_new_data_cv.notify_one();
    }
    //}

    /* data_callback() method //{ */
    virtual void data_callback(const typename MessageType::ConstSharedPtr& msg)
    {
      {
        std::lock_guard lck(m_new_data_mtx);
        if (m_timeout_manager && m_timeout_id.has_value())
          m_timeout_manager->reset(m_timeout_id.value(), m_node->get_clock()->now());
        process_new_message(msg);
      }

      // execute the callback after unlocking the mutex to enable multi-threaded callback execution
      if (m_message_callback)
        m_message_callback(msg);
    }
    //}
  };
  //}

  /* SubscribeHandler_threadsafe class //{ */
  template <typename MessageType>
  class SubscriberHandler<MessageType>::ImplThreadsafe : public SubscriberHandler<MessageType>::Impl
  {
  private:
    using impl_class_t = SubscriberHandler<MessageType>::Impl;

  public:
    using timeout_callback_t = typename impl_class_t::timeout_callback_t;
    using message_callback_t = typename impl_class_t::message_callback_t;

    friend class SubscriberHandler<MessageType>;

  public:
    ImplThreadsafe(const SubscriberHandlerOptions& options, const message_callback_t& message_callback = message_callback_t())
        : impl_class_t::Impl(options, message_callback)
    {
    }

  public:
    virtual bool hasMsg() const override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::hasMsg();
    }
    virtual bool newMsg() const override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::newMsg();
    }
    virtual bool usedMsg() const override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::usedMsg();
    }
    virtual typename MessageType::ConstSharedPtr getMsg() override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::getMsg();
    }
    virtual typename MessageType::ConstSharedPtr peekMsg() const override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::peekMsg();
    }
    virtual rclcpp::Time lastMsgTime() const override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::lastMsgTime();
    };
    virtual std::string topicName() const override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::topicName();
    };
    virtual void start() override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::start();
    }
    virtual void stop() override
    {
      std::lock_guard lck(m_mtx);
      return impl_class_t::stop();
    }

    virtual ~ImplThreadsafe() override = default;

  protected:
    virtual void data_callback(const typename MessageType::ConstSharedPtr& msg) override
    {
      {
        std::scoped_lock lck(m_mtx, this->m_new_data_mtx);

        if (this->m_timeout_manager && this->m_timeout_id.has_value())
        {
          this->m_timeout_manager->reset(this->m_timeout_id.value(), this->m_node->get_clock()->now());
        }

        impl_class_t::process_new_message(msg);
      }

      // execute the callback after unlocking the mutex to enable multi-threaded callback execution
      if (this->m_message_callback)
        impl_class_t::m_message_callback(msg);
    }

  private:
    mutable std::recursive_mutex m_mtx;
  };
  //}

}  // namespace mrs_lib

#endif  // SUBSCRIBER_HANDLER_HPP
