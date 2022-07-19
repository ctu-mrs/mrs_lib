// clang: MatousFormat

#ifndef SUBSCRIBE_HANDLER_HPP
#define SUBSCRIBE_HANDLER_HPP

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/timer.h>
#include <mutex>
#include <condition_variable>

namespace mrs_lib
{
  /* SubscribeHandler::Impl class //{ */
  // implements the constructor, getMsg() method and data_callback method (non-thread-safe)
  template <typename MessageType>
  class SubscribeHandler<MessageType>::Impl
  {
  public:
    using timeout_callback_t = typename SubscribeHandler<MessageType>::timeout_callback_t;
    using message_callback_t = typename SubscribeHandler<MessageType>::message_callback_t;
    using data_callback_t = std::function<void(const typename MessageType::ConstPtr&)>;

  private:
    friend class SubscribeHandler<MessageType>;

  public:
    /* constructor //{ */
    Impl(SubscribeHandler* owner, const SubscribeHandlerOptions& options, const message_callback_t& message_callback = message_callback_t())
        : m_owner(owner),
          m_nh(options.nh),
          m_no_message_timeout(options.no_message_timeout),
          m_topic_name(options.topic_name),
          m_node_name(options.node_name),
          m_got_data(false),
          m_new_data(false),
          m_used_data(false),
          m_last_msg_received(ros::Time::now()),
          m_timeout_callback(options.timeout_callback),
          m_latest_message(nullptr),
          m_message_callback(message_callback),
          m_queue_size(options.queue_size),
          m_transport_hints(options.transport_hints)
    {
      if (!m_timeout_callback)
        m_timeout_callback = std::bind(&Impl::default_timeout_callback, this, std::placeholders::_1, std::placeholders::_2,
                                       std::placeholders::_3);

      if (options.no_message_timeout != mrs_lib::no_timeout)
      {
        if (options.use_thread_timer)
          m_timeout_check_timer = std::make_unique<mrs_lib::ThreadTimer>(m_nh, options.no_message_timeout, &Impl::check_timeout, this, true /*oneshot*/, false /*autostart*/);
        else
          m_timeout_check_timer = std::make_unique<mrs_lib::ROSTimer>(m_nh, options.no_message_timeout, &Impl::check_timeout, this, true /*oneshot*/, false /*autostart*/);
      }

      const std::string msg = "Subscribed to topic '" + m_topic_name + "' -> '" + topicName() + "'";
      if (m_node_name.empty())
        ROS_INFO_STREAM(msg);
      else
        ROS_INFO_STREAM("[" << m_node_name << "]: " << msg);
    }
    //}

    virtual ~Impl() = default;

  public:
    /* getMsg() method //{ */
    virtual typename MessageType::ConstPtr getMsg()
    {
      m_new_data = false;
      m_used_data = true;
      return peekMsg();
    }
    //}

    /* peekMsg() method //{ */
    virtual typename MessageType::ConstPtr peekMsg() const
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
    virtual typename MessageType::ConstPtr waitForNew(const ros::WallDuration& timeout)
    {
      // convert the ros type to chrono type
      const std::chrono::duration<float> chrono_timeout(timeout.toSec());
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
    virtual ros::Time lastMsgTime() const
    {
      std::lock_guard lck(m_last_msg_received_mtx);
      return m_last_msg_received;
    };
    //}

    /* topicName() method //{ */
    virtual std::string topicName() const
    {
      std::string ret = m_sub.getTopic();
      if (ret.empty())
        ret = m_nh.resolveName(m_topic_name);
      return ret;
    }
    //}

    /* start() method //{ */
    virtual void start()
    {
      if (m_timeout_check_timer)
        m_timeout_check_timer->start();
      m_sub = m_nh.subscribe(m_topic_name, m_queue_size, &Impl::data_callback, this, m_transport_hints);
    }
    //}

    /* stop() method //{ */
    virtual void stop()
    {
      if (m_timeout_check_timer)
        m_timeout_check_timer->stop();
      m_sub.shutdown();
    }
    //}

  protected:
    /* check_time_reset() method //{ */
    bool check_time_reset(const ros::Time& now)
    {
      return now < m_last_msg_received;
    }
    //}

  protected:
    SubscribeHandler* m_owner;
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;

  protected:
    ros::Duration m_no_message_timeout;
    std::string m_topic_name;
    std::string m_node_name;

  protected:
    bool m_got_data;   // whether any data was received

    mutable std::mutex m_new_data_mtx;
    mutable std::condition_variable m_new_data_cv;
    bool m_new_data;   // whether new data was received since last call to get_data

    bool m_used_data;  // whether get_data was successfully called at least once

  protected:
    mutable std::mutex m_last_msg_received_mtx;
    ros::Time m_last_msg_received;
    std::unique_ptr<mrs_lib::MRSTimer> m_timeout_check_timer;
    timeout_callback_t m_timeout_callback;

  protected:
    typename MessageType::ConstPtr m_latest_message;
    message_callback_t m_message_callback;

  private:
    uint32_t m_queue_size;
    ros::TransportHints m_transport_hints;

  protected:
    /* default_timeout_callback() method //{ */
    void default_timeout_callback(const std::string& topic, const ros::Time& last_msg, const int n_pubs)
    {
      /* ROS_ERROR("Checking topic %s, delay: %.2f", m_sub.getTopic().c_str(), since_msg.toSec()); */
      ros::Duration since_msg = (ros::Time::now() - last_msg);
      const std::string msg = "Did not receive any message from topic '" + topic + "' for " + std::to_string(since_msg.toSec()) + "s ("
                              + std::to_string(n_pubs) + " publishers on this topic)";
      if (m_node_name.empty())
        ROS_WARN_STREAM(msg);
      else
        ROS_WARN_STREAM("[" << m_node_name << "]: " << msg);
    }
    //}

    /* check_timeout() method //{ */
    void check_timeout([[maybe_unused]] const ros::TimerEvent& evt)
    {
      if (m_timeout_check_timer)
        m_timeout_check_timer->stop();
      ros::Time last_msg;
      {
        std::lock_guard lck(m_last_msg_received_mtx);
        last_msg = m_last_msg_received;
      }
      const auto n_pubs = m_sub.getNumPublishers();
      if (m_timeout_check_timer)
        m_timeout_check_timer->start();
      m_timeout_callback(topicName(), last_msg, n_pubs);
    }
    //}

    /* process_new_message() method //{ */
    void process_new_message(const typename MessageType::ConstPtr& msg)
    {
      m_latest_message = msg;
      m_new_data = true;
      m_got_data = true;
      m_last_msg_received = ros::Time::now();
      m_new_data_cv.notify_one();
    }
    //}

    /* data_callback() method //{ */
    virtual void data_callback(const typename MessageType::ConstPtr& msg)
    {
      if (m_timeout_check_timer)
        m_timeout_check_timer->stop();
      std::lock_guard lck(m_new_data_mtx);
      process_new_message(msg);
      if (m_message_callback)
        m_message_callback(*m_owner);
      if (m_timeout_check_timer)
        m_timeout_check_timer->start();
    }
    //}
  };
  //}

  /* SubscribeHandler_threadsafe class //{ */
  template <typename MessageType>
  class SubscribeHandler<MessageType>::ImplThreadsafe : public SubscribeHandler<MessageType>::Impl
  {
  private:
    using impl_class_t = SubscribeHandler<MessageType>::Impl;

  public:
    using timeout_callback_t = typename impl_class_t::timeout_callback_t;
    using message_callback_t = typename impl_class_t::message_callback_t;

    friend class SubscribeHandler<MessageType>;

  public:
    ImplThreadsafe(SubscribeHandler* owner, const SubscribeHandlerOptions& options, const message_callback_t& message_callback = message_callback_t())
        : impl_class_t::Impl(owner, options, message_callback)
    {
    }

  public:
    virtual bool hasMsg() const override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::hasMsg();
    }
    virtual bool newMsg() const override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::newMsg();
    }
    virtual bool usedMsg() const override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::usedMsg();
    }
    virtual typename MessageType::ConstPtr getMsg() override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::getMsg();
    }
    virtual typename MessageType::ConstPtr peekMsg() const override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::peekMsg();
    }
    virtual ros::Time lastMsgTime() const override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::lastMsgTime();
    };
    virtual std::string topicName() const override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::topicName();
    };
    virtual void start() override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::start();
    }
    virtual void stop() override
    {
      std::lock_guard<std::recursive_mutex> lck(m_mtx);
      return impl_class_t::stop();
    }

    virtual ~ImplThreadsafe() override = default;

  protected:
    virtual void data_callback(const typename MessageType::ConstPtr& msg) override
    {
      // the stop has to come before the mutex lock to enable the timer callbacks currently
      // in the queue to execute and prevent deadlock (.stop() waits for all currently
      // queued callbacks to finish before returning...)
      if (this->m_timeout_check_timer)
        this->m_timeout_check_timer->stop();
      std::scoped_lock lck(m_mtx, this->m_new_data_mtx);
      this->process_new_message(msg);
      if (this->m_message_callback)
        this->m_message_callback(*(this->m_owner));
      if (this->m_timeout_check_timer)
        this->m_timeout_check_timer->start();
    }

  private:
    mutable std::recursive_mutex m_mtx;
  };
  //}

}  // namespace mrs_lib

#endif  // SUBSCRIBE_HANDLER_HPP
