// clang: MatousFormat
/**  \file
     \brief Defines SubscribeMgr and related convenience classes for subscribing ROS topics.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef SUBRSCRIBE_HANDLER_H
#define SUBRSCRIBE_HANDLER_H

#include <ros/ros.h>

namespace mrs_lib
{
  static const ros::Duration no_timeout = ros::Duration(0);

  template <typename MessageType>
  class SubscribeHandler;

/*!
  * \brief Helper typedef for a more convenient usage of the shared_ptrs for the SubscribeHandler.
  */
  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType>>;

  struct SubscribeHandlerOptions
  {
    ros::NodeHandle nh;
    std::string node_name = {};
    ros::Duration no_message_timeout = mrs_lib::no_timeout;
    bool threadsafe = true;
    bool autostart = true;
    uint32_t queue_size = 10;
    ros::TransportHints transport_hints = ros::TransportHints();

    std::string topic_name = {};
    std::function<void(const std::string&, const ros::Time&, const int)> timeout_callback = {};
  };

  template <typename MessageType>
  struct MessageWrapper
  {
    public:
      MessageWrapper(const typename MessageType::ConstPtr msg_ptr) : m_msg_ptr(msg_ptr), m_msg_used(false) {}
      typename MessageType::ConstPtr get_data() {m_msg_used = true; return m_msg_ptr;}
      typename MessageType::ConstPtr peek_data() {return m_msg_ptr;}
      bool used_data() {return m_msg_used;}
    private:
      const typename MessageType::ConstPtr m_msg_ptr;
      bool m_msg_used;
  };
}

#include <impl/subscribe_handler.hpp>

namespace mrs_lib
{
  /* SubscribeHandler class //{ */
  /**
  * \brief Base class for pointers, returned by the SubscribeMgr.
  *
  * For example of instantiation and usage of this class, see documentation of SubscribeMgr.
  *
  * \warning This class should not be manually instantiated by the user. Instead, use this class to define smart pointers, returned by the SubscribeMgr::create_handler() method.
  *
  */
  template <typename MessageType>
  class SubscribeHandler
  {
    public:
    /*!
      * \brief Convenience type for the template parameter to enable nice aliasing.
      */
      using message_type = MessageType;

    /*!
      * \brief Type for the timeout callback function. For clarity and consistency, it is recommended to use the SubscribeMgr::timeout_callback_t instead.
      */
      using timeout_callback_t = std::function<void(const std::string&, const ros::Time&, const int)>;

    /*!
      * \brief Convenience type for the message callback function.
      */
      using message_callback_t = std::function<void(MessageWrapper<MessageType>&)>;

    public:
    /*!
      * \brief Returns the last received message on the topic, handled by this SubscribeHandler.
      *
      * \return the last received message.
      */
      typename MessageType::ConstPtr get_data() {assert(m_pimpl); return m_pimpl->get_data();};

    /*!
      * \brief Returns the last received message on the topic without resetting the new_data() or used_data() flags.
      *
      * \return the last received message.
      */
      typename MessageType::ConstPtr peek_data() {assert(m_pimpl); return m_pimpl->peek_data();};

    /*!
      * \brief Used to check whether at least one message has been received on the handled topic.
      *
      * \return true if at least one message was received, otherwise false.
      */
      bool has_data() const {assert(m_pimpl); return m_pimpl->has_data();};

    /*!
      * \brief Used to check whether at least one message has been received on the handled topic since the last call to get_data().
      *
      * \return true if at least one message was received, otherwise false.
      */
      bool new_data() const {assert(m_pimpl); return m_pimpl->new_data();};

    /*!
      * \brief Used to check whether get_data() was called at least once on this SubscribeHandler.
      *
      * \return true if get_data() was called at least once, otherwise false.
      */
      bool used_data() const {assert(m_pimpl); return m_pimpl->used_data();};

    /*!
      * \brief Returns time of the last received message on the topic, handled by this SubscribeHandler.
      *
      * \return time when the last message was received.
      */
      ros::Time last_message_time() const {assert(m_pimpl); return m_pimpl->last_message_time();};

    /*!
      * \brief Enables the callbacks for the handled topic.
      *
      * If the SubscribeHandler object is stopped using the stop() method, no callbacks will be called
      * until the start() method is called.
      */
      void start() const {assert(m_pimpl); return m_pimpl->start();};

    /*!
      * \brief Disables the callbacks for the handled topic.
      *
      * All messages after this method is called will be ignored until start() is called again.
      * Timeout checking will also be disabled.
      */
      void stop() const {assert(m_pimpl); return m_pimpl->stop();};

    public:
      SubscribeHandler() {};

    /*!
      * \brief Main constructor.
      *
      */
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            const message_callback_t& message_callback = {}
          )
      {
        if (options.threadsafe)
        {
          m_pimpl = std::make_unique<impl::SubscribeHandler_threadsafe<MessageType>>
            (
              options,
              message_callback
            );
        }
        else
        {
          m_pimpl = std::make_unique<impl::SubscribeHandler_impl<MessageType>>
            (
              options,
              message_callback
            );
        }
        m_pimpl->template set_data_callback<false>();
        if (options.autostart)
          start();
      };

    /*!
      * \brief Convenience constructor overload.
      *
      */
      template <class ... Types>
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            const timeout_callback_t& timeout_callback,
            Types ... args
          )
      :
        SubscribeHandler(
            [options, timeout_callback]()
            {
              SubscribeHandlerOptions opts = options;
              opts.timeout_callback = timeout_callback;
              return opts;
            }(),
            args...
            )
      {
      };

    /*!
      * \brief Convenience constructor overload.
      *
      */
      template <class ObjectType1, class ... Types>
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            void (ObjectType1::*const timeout_callback) (const std::string&, const ros::Time&, const int),
            ObjectType1* const obj1,
            Types ... args
          )
      :
        SubscribeHandler(
            [options, timeout_callback, obj1]()
            {
              SubscribeHandlerOptions opts = options;
              opts.timeout_callback = timeout_callback == nullptr ? timeout_callback_t() : std::bind(timeout_callback, obj1, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
              return opts;
            }(),
            args...
            )
      {
      };

    /*!
      * \brief Convenience constructor overload.
      *
      */
      template <class ObjectType2, class ... Types>
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            void (ObjectType2::*const message_callback) (MessageWrapper<MessageType>&),
            ObjectType2* const obj2,
            Types ... args
          )
      :
        SubscribeHandler(
            options,
            message_callback == nullptr ? message_callback_t() : std::bind(message_callback, obj2, std::placeholders::_1),
            args...
            )
      {
      };

    /*!
      * \brief Convenience constructor overload.
      *
      */
     template <class ObjectType1, class ObjectType2, class ... Types>
     SubscribeHandler(
           const SubscribeHandlerOptions& options,
           void (ObjectType2::*const message_callback) (MessageWrapper<MessageType>&),
           ObjectType2* const obj2,
           void (ObjectType1::*const timeout_callback) (const std::string&, const ros::Time&, const int),
           ObjectType1* const obj1,
           Types ... args
         )
     :
       SubscribeHandler(
            [options, timeout_callback, obj1]()
            {
              SubscribeHandlerOptions opts = options;
              opts.timeout_callback = timeout_callback == nullptr ? timeout_callback_t() : std::bind(timeout_callback, obj1, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
              return opts;
            }(),
            message_callback == nullptr ? message_callback_t() : std::bind(message_callback, obj2, std::placeholders::_1),
            args...
            )
     {
     };

    /*!
      * \brief Convenience constructor overload.
      *
      */
      template<class ... Types>
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            const ros::Duration& no_message_timeout,
            Types ... args
          )
      :
        SubscribeHandler(
            [options, no_message_timeout]()
            {
              SubscribeHandlerOptions opts = options;
              opts.no_message_timeout = no_message_timeout;
              return opts;
            }(),
            args...
            )
      {
      };

    /*!
      * \brief Convenience constructor overload.
      *
      */
      template<class ... Types>
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            const std::string& topic_name,
            Types ... args
          )
      :
        SubscribeHandler(
            [options, topic_name]()
            {
              SubscribeHandlerOptions opts = options;
              opts.topic_name = topic_name;
              return opts;
            }(),
            args...
            )
      {
      };

    private:
      std::unique_ptr<impl::SubscribeHandler_impl<MessageType>> m_pimpl;
  };
  //}

/*!
  * \brief Helper alias for convenient extraction of handled message type from a SubscribeHandlerPtr.
  */
  template<typename SubscribeHandler>
  using message_type = typename SubscribeHandler::message_type;

  template<typename Class, class ... Types>
  void construct_object(
        Class& object,
        Types ... args
      )
  {
    object = Class(args...);
  };
}

#endif // SUBRSCRIBE_HANDLER_H
