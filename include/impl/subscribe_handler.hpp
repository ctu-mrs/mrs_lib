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
    Impl(const SubscribeHandlerOptions& options, const message_callback_t& message_callback = message_callback_t())
        : m_nh(options.nh),
          m_topic_name(options.topic_name),
          m_node_name(options.node_name),
          m_got_data(false),
          m_new_data(false),
          m_used_data(false),
          m_timeout_manager(options.timeout_manager),
          m_latest_message(nullptr),
          m_message_callback(message_callback),
          m_queue_size(options.queue_size),
          m_transport_hints(options.transport_hints)
    {
      timeout_callback_t timeout_callback = options.timeout_callback;
      if (!timeout_callback)
        timeout_callback = std::bind(&Impl::default_timeout_callback, this, std::placeholders::_1);

      if (options.no_message_timeout != mrs_lib::no_timeout)
      {
        if (!m_timeout_manager)
          m_timeout_manager = std::make_shared<mrs_lib::TimeoutManager>(m_nh, ros::Rate(2.0/options.no_message_timeout.toSec()));
        m_timeout_id = m_timeout_manager->registerNew(options.no_message_timeout, timeout_callback);
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
      return m_timeout_manager->lastReset(m_timeout_id);
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
      if (m_timeout_manager)
        m_timeout_manager->start(m_timeout_id);
      m_sub = m_nh.subscribe(m_topic_name, m_queue_size, &Impl::data_callback, this, m_transport_hints);
    }
    //}

    /* stop() method //{ */
    virtual void stop()
    {
      if (m_timeout_manager)
        m_timeout_manager->pause(m_timeout_id);
      m_sub.shutdown();
    }
    //}

  protected:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;

  protected:
    std::string m_topic_name;
    std::string m_node_name;

  protected:
    bool m_got_data;   // whether any data was received

    mutable std::mutex m_new_data_mtx;
    mutable std::condition_variable m_new_data_cv;
    bool m_new_data;   // whether new data was received since last call to get_data

    bool m_used_data;  // whether get_data was successfully called at least once

  protected:
    std::shared_ptr<mrs_lib::TimeoutManager> m_timeout_manager;
    mrs_lib::TimeoutManager::timeout_id_t m_timeout_id;

  protected:
    typename MessageType::ConstPtr m_latest_message;
    message_callback_t m_message_callback;

  private:
    uint32_t m_queue_size;
    ros::TransportHints m_transport_hints;

  protected:
    /* default_timeout_callback() method //{ */
    void default_timeout_callback(const ros::Time& last_msg)
    {
      const ros::Duration since_msg = (ros::Time::now() - last_msg);
      const auto n_pubs = m_sub.getNumPublishers();
      const std::string txt = "Did not receive any message from topic '" + m_topic_name + "' for " + std::to_string(since_msg.toSec()) + "s ("
                              + std::to_string(n_pubs) + " publishers on this topic)";
      if (m_node_name.empty())
        ROS_WARN_STREAM(txt);
      else
        ROS_WARN_STREAM("[" << m_node_name << "]: " << txt);
    }
    //}

    /* process_new_message() method //{ */
    void process_new_message(const typename MessageType::ConstPtr& msg)
    {
      m_latest_message = msg;
      m_new_data = true;
      m_got_data = true;
      m_new_data_cv.notify_one();
    }
    //}

    /* data_callback() method //{ */
    virtual void data_callback(const typename MessageType::ConstPtr& msg)
    {
      if (m_timeout_manager)
        m_timeout_manager->reset(m_timeout_id);
      std::lock_guard lck(m_new_data_mtx);
      process_new_message(msg);
      if (m_message_callback)
        m_message_callback(msg);
    }
    //}
  };
  //}

}  // namespace mrs_lib

#endif  // SUBSCRIBE_HANDLER_HPP
