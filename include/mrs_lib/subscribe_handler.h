// clang: MatousFormat
/**  \file
     \brief Defines SubscribeHandler and related convenience classes for subscribing to ROS topics.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef SUBRSCRIBE_HANDLER_H
#define SUBRSCRIBE_HANDLER_H

#include <ros/ros.h>

namespace mrs_lib
{

  static const ros::Duration no_timeout = ros::Duration(0);

  /* struct SubscribeHandlerOptions //{ */
  
  /**
  * \brief A helper class to simplify setup of SubscribeHandler construction.
  * This class is passed to the SubscribeHandler constructor and specifies its common options.
  *
  * \note Any option, passed directly to the SubscribeHandler constructor outside this structure, *OVERRIDES* values in this structure.
  * The values in this structure can be thought of as default common values for all SubscribeHandler objects you want to create,
  * and values passed directly to the constructor as specific options for the concrete handler.
  *
  */
  struct SubscribeHandlerOptions
  {
    SubscribeHandlerOptions(const ros::NodeHandle& nh) : nh(nh) {}
    SubscribeHandlerOptions() = default;
  
    ros::NodeHandle nh;  /*!< \brief The ROS NodeHandle to be used for subscription. */
  
    std::string node_name = {};  /*!< \brief Name of the ROS node, using this handle (used for messages printed to console). */
  
    std::string topic_name = {};  /*!< \brief Name of the ROS topic to be handled. */
  
    ros::Duration no_message_timeout = mrs_lib::no_timeout;  /*!< \brief If no new message is received for this duration, the \p timeout_callback function will be called. If \p timeout_callback is empty, an error message will be printed to the console. */
  
    bool use_thread_timer = false;  /*!< \brief Selects whether to use an STL thread-based timer implementation instead of ROS' own. */
  
    std::function<void(const std::string&, const ros::Time&, const int)> timeout_callback = {};  /*!< \brief This function will be called if no new message is received for the \p no_message_timeout duration. If this variable is empty, an error message will be printed to the console. */
  
    bool threadsafe = true;  /*!< \brief If true, all methods of the SubscribeHandler will be mutexed (using a recursive mutex) to avoid data races. */
  
    bool autostart = true;  /*!< \brief If true, the SubscribeHandler will be started after construction. Otherwise it has to be started using the start() method */
  
    uint32_t queue_size = 3;  /*!< \brief This parameter is passed to the NodeHandle when subscribing to the topic */
  
    ros::TransportHints transport_hints = ros::TransportHints();  /*!< \brief This parameter is passed to the NodeHandle when subscribing to the topic */
  };
  
  //}

  /* SubscribeHandler class //{ */
  /**
  * \brief The main class for ROS topic subscription, message timeout handling etc.
  *
  * This class handles the raw ROS Subscriber for a specified topic. The last message received on the topic is remembered
  * and may be retrieved by calling the getMsg() method. To check whether at least one message was received, use hasMsg()
  * (if no message was received and you call getMsg(), a nullptr is returned and an error message is printed). To check
  * whether a new message has arrived since the last call to getMsg(), use newMsg() (useful to check whether a new message
  * needs to be processed in a loop or ROS Timer callback).
  *
  * A timeout callback function may be specified, which is called if no message is received on the topic for a specified
  * timeout (use the \p timeout_callback and \p no_message_timeout parameters). If the timeout callback is not set by the
  * user, an error message is printed to the console after the timeout by default.
  *
  * A message callback function may be specified, which is called whenever a new message is received (use the
  * \p message_callback parameter).
  *
  * The callbacks and timeouts may be stopped and started using the stop() and start() methods.
  *
  * For more details, see the example below (I recommend starting with the simple_example which covers most use-cases).
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
      * \brief Type for the timeout callback function.
      */
      using timeout_callback_t = std::function<void(const std::string&, const ros::Time&, const int)>;

    /*!
      * \brief Convenience type for the message callback function.
      */
      using message_callback_t = std::function<void(SubscribeHandler<MessageType>&)>;

    public:
    /*!
      * \brief Returns the last received message on the topic, handled by this SubscribeHandler.
      * Use hasMsg() first to check if at least one message is available or newMsg() to check if a new message
      * since the last call to getMsg() is available.
      *
      * \return the last received message.
      */
      virtual typename MessageType::ConstPtr getMsg() {assert(m_pimpl); return m_pimpl->getMsg();};

    /*!
      * \brief Returns the last received message on the topic without modifying the newMsg() or usedMsg() flags.
      *
      * \return the last received message.
      */
      virtual typename MessageType::ConstPtr peekMsg() const {assert(m_pimpl); return m_pimpl->peekMsg();};

    /*!
      * \brief Used to check whether at least one message has been received on the handled topic.
      *
      * \return true if at least one message was received, otherwise false.
      */
      virtual bool hasMsg() const {assert(m_pimpl); return m_pimpl->hasMsg();};

    /*!
      * \brief Used to check whether at least one message has been received on the handled topic since the last call to getMsg().
      *
      * \return true if at least one message was received, otherwise false.
      */
      virtual bool newMsg() const {assert(m_pimpl); return m_pimpl->newMsg();};

    /*!
      * \brief Used to check whether getMsg() was called at least once on this SubscribeHandler.
      *
      * \return true if getMsg() was called at least once, otherwise false.
      */
      virtual bool usedMsg() const {assert(m_pimpl); return m_pimpl->usedMsg();};

    /*!
      * \brief Blocks until new data becomes available or until the timeout runs out or until a spurious wake-up.
      *
      * \return the message if a new message is available after waking up, \p nullptr otherwise.
      */
      virtual typename MessageType::ConstPtr waitForNew(const ros::WallDuration& timeout) {assert(m_pimpl); return m_pimpl->waitForNew(timeout);};

    /*!
      * \brief Returns time of the last received message on the topic, handled by this SubscribeHandler.
      *
      * \return time when the last message was received.
      */
      virtual ros::Time lastMsgTime() const {assert(m_pimpl); return m_pimpl->lastMsgTime();};

    /*!
      * \brief Returns the resolved (full) name of the topic, handled by this SubscribeHandler.
      *
      * \return name of the handled topic.
      */
      virtual std::string topicName() const {assert(m_pimpl); return m_pimpl->topicName();};

    /*!
      * \brief Returns the subscribed (unresolved) name of the topic, handled by this SubscribeHandler.
      *
      * \return name of the handled topic.
      */
      virtual std::string subscribedTopicName() const {assert(m_pimpl); return m_pimpl->m_topic_name;};

    /*!
      * \brief Enables the callbacks for the handled topic.
      *
      * If the SubscribeHandler object is stopped using the stop() method, no callbacks will be called
      * until the start() method is called.
      */
      virtual void start() {assert(m_pimpl); return m_pimpl->start();};

    /*!
      * \brief Disables the callbacks for the handled topic.
      *
      * All messages after this method is called will be ignored until start() is called again.
      * Timeout checking will also be disabled.
      */
      virtual void stop() {assert(m_pimpl); return m_pimpl->stop();};

    public:
    /*!
      * \brief Default constructor to avoid having to use pointers.
      *
      * It does nothing and the SubscribeHandler it constructs will also do nothing.
      * Use some of the other constructors for actual construction of a usable SubscribeHandler.
      */
      SubscribeHandler() {};

    /*!
      * \brief Main constructor.
      *
      * \param options    The common options struct (see documentation of SubscribeHandlerOptions).
      * \param topic_name Name of the topic to be handled by this subscribed.
      * \param args       Remaining arguments to be parsed (see other constructors).
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
      }

    /*!
      * \brief Convenience constructor overload.
      *
      * \param options          The common options struct (see documentation of SubscribeHandlerOptions).
      * \param message_callback The callback function to call when a new message is received (you can leave this argument empty and just use the newMsg()/hasMsg() and getMsg() interface).
      *
      */
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            const message_callback_t& message_callback = {}
          )
      {
        if (options.threadsafe)
        {
          m_pimpl = std::make_unique<ImplThreadsafe>
            (
              this,
              options,
              message_callback
            );
        }
        else
        {
          m_pimpl = std::make_unique<Impl>
            (
              this,
              options,
              message_callback
            );
        }
        if (options.autostart)
          start();
      };

    /*!
      * \brief Convenience constructor overload.
      *
      * \param options          The common options struct (see documentation of SubscribeHandlerOptions).
      * \param timeout_callback The callback function to call when a new message is not received for the duration, specified in \p options or in the \p no_message_timeout parameter.
      * \param args             Remaining arguments to be parsed (see other constructors).
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
      }

    /*!
      * \brief Convenience constructor overload.
      *
      * \param options          The common options struct (see documentation of SubscribeHandlerOptions).
      * \param timeout_callback The callback method to call when a new message is not received for the duration, specified in \p options or in the \p no_message_timeout parameter.
      * \param obj1             The object on which the callback method \p timeout_callback will be called.
      * \param args             Remaining arguments to be parsed (see other constructors).
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
      }

    /*!
      * \brief Convenience constructor overload.
      *
      * \param options          The common options struct (see documentation of SubscribeHandlerOptions).
      * \param message_callback The callback method to call when a new message is received.
      * \param obj2             The object on which the callback method \p timeout_callback will be called.
      * \param args             Remaining arguments to be parsed (see other constructors).
      *
      */
      template <class ObjectType2, class ... Types>
      SubscribeHandler(
            const SubscribeHandlerOptions& options,
            void (ObjectType2::*const message_callback) (SubscribeHandler<MessageType>&),
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
      }

    /*!
      * \brief Convenience constructor overload.
      *
      * \param options          The common options struct (see documentation of SubscribeHandlerOptions).
      * \param message_callback The callback method to call when a new message is received.
      * \param obj2             The object on which the callback method \p timeout_callback will be called.
      * \param timeout_callback The callback method to call when a new message is not received for the duration, specified in \p options or in the \p no_message_timeout parameter.
      * \param obj1             The object on which the callback method \p timeout_callback will be called.
      * \param args             Remaining arguments to be parsed (see other constructors).
      *
      */
     template <class ObjectType1, class ObjectType2, class ... Types>
     SubscribeHandler(
           const SubscribeHandlerOptions& options,
           void (ObjectType2::*const message_callback) (SubscribeHandler<MessageType>&),
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
     }

    /*!
      * \brief Convenience constructor overload.
      *
      * \param options            The common options struct (see documentation of SubscribeHandlerOptions).
      * \param no_message_timeout If no message is received for this duration, the timeout callback function is called. If no timeout callback is specified, an error message is printed to the console.
      * \param args               Remaining arguments to be parsed (see other constructors).
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
      }

      ~SubscribeHandler() = default;
      // delete copy constructor and assignment operator (forbid copying shandlers)
      SubscribeHandler(const SubscribeHandler&) = delete;
      SubscribeHandler& operator=(const SubscribeHandler&) = delete;
      // define only move constructor and assignemnt operator
      SubscribeHandler(SubscribeHandler&& other)
      {
        this->m_pimpl = std::move(other.m_pimpl);
        other.m_pimpl = nullptr;
        this->m_pimpl->m_owner = this;
      }
      SubscribeHandler& operator=(SubscribeHandler&& other)
      {
        this->m_pimpl = std::move(other.m_pimpl);
        other.m_pimpl = nullptr;
        this->m_pimpl->m_owner = this;
        return *this;
      }

    private:
      class Impl;
      class ImplThreadsafe;
      std::unique_ptr<Impl> m_pimpl;
  };
  //}

/*!
  * \brief Helper alias for convenient extraction of handled message type from a SubscribeHandlerPtr.
  */
  template<typename SubscribeHandler>
  using message_type = typename SubscribeHandler::message_type;

/*!
  * \brief Helper function for object construstion e.g. in case of member objects.
  * This function is useful to avoid specifying object template parameters twice - once in definition of the variable and second time during object construction.
  * This function can deduce the template parameters from the type of the already defined object, because it returns the newly constructed object as a reference
  * argument and not as a return type.
  *
  * \param object The object to be constructed.
  * \param args   These arguments are passed to the object constructor.
  */
  template<typename Class, class ... Types>
  void construct_object(
        Class& object,
        Types ... args
      )
  {
    object = Class(args...);
  }
}

#include <impl/subscribe_handler.hpp>

#endif // SUBRSCRIBE_HANDLER_H
