// clang: MatousFormat
/**  \file
     \brief Defines SubscriberHandler and related convenience classes for subscribing to ROS topics.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef SUBRSCRIBER_HANDLER_H
#define SUBRSCRIBER_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time_source.hpp>

#include <mrs_lib/timeout_manager.h>

namespace mrs_lib
{

  static const rclcpp::Duration no_timeout = rclcpp::Duration(0, 0);

  /* struct SubscriberHandlerOptions //{ */

  /**
   * \brief A helper class to simplify setup of SubscriberHandler construction.
   * This class is passed to the SubscriberHandler constructor and specifies its common options.
   *
   * \note Any option, passed directly to the SubscriberHandler constructor outside this structure, *OVERRIDES* values in this structure.
   * The values in this structure can be thought of as default common values for all SubscriberHandler objects you want to create,
   * and values passed directly to the constructor as specific options for the concrete handler.
   *
   */
  struct SubscriberHandlerOptions
  {
    SubscriberHandlerOptions(const rclcpp::Node::SharedPtr& node) : node(node)
    {
    }
    SubscriberHandlerOptions() = default;

    rclcpp::Node::SharedPtr node; /*!< \brief The ROS NodeHandle to be used for subscription. */

    std::string node_name = {}; /*!< \brief Name of the ROS node, using this handle (used for messages printed to console). */

    std::string topic_name = {}; /*!< \brief Name of the ROS topic to be handled. */

    std::shared_ptr<mrs_lib::TimeoutManager> timeout_manager = nullptr; /*!< \brief Will be used for handling message timouts if necessary. If no manager is
                                                                           specified, it will be created with rate equal to half of \p no_message_timeout. */

    rclcpp::Duration no_message_timeout =
        mrs_lib::no_timeout; /*!< \brief If no new message is received for this duration, the \p timeout_callback function will
                             be called. If \p timeout_callback is empty, an error message will be printed to the console. */

    std::function<void(const std::string& topic_name, const rclcpp::Time& last_msg)> timeout_callback =
        {}; /*!< \brief This function will be called if no new message is received for the \p no_message_timeout duration. If this variable is empty, an error
               message will be printed to the console. */

    bool threadsafe = true; /*!< \brief If true, all methods of the SubscriberHandler will be mutexed (using a recursive mutex) to avoid data races. */

    bool autostart =
        true; /*!< \brief If true, the SubscriberHandler will be started after construction. Otherwise it has to be started using the start() method */

    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    rclcpp::SubscriptionOptions subscription_options = rclcpp::SubscriptionOptions();
  };

  //}

  /* SubscriberHandler class //{ */
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
  class SubscriberHandler
  {
  public:
    /*!
     * \brief Convenience type for the template parameter to enable nice aliasing.
     */
    using message_type = MessageType;

    /*!
     * \brief Type for the timeout callback function.
     */
    using timeout_callback_t = std::function<void(const std::string& topic_name, const rclcpp::Time& last_msg)>;

    /*!
     * \brief Convenience type for the message callback function.
     */
    using message_callback_t = std::function<void(typename MessageType::ConstSharedPtr)>;

  public:
    /*!
     * \brief Returns the last received message on the topic, handled by this SubscriberHandler.
     * Use hasMsg() first to check if at least one message is available or newMsg() to check if a new message
     * since the last call to getMsg() is available.
     *
     * \return the last received message.
     */
    virtual typename MessageType::ConstSharedPtr getMsg()
    {
      assert(m_pimpl);
      return m_pimpl->getMsg();
    };

    /*!
     * \brief Returns the last received message on the topic without modifying the newMsg() or usedMsg() flags.
     *
     * \return the last received message.
     */
    virtual typename MessageType::ConstSharedPtr peekMsg() const
    {
      assert(m_pimpl);
      return m_pimpl->peekMsg();
    };

    /*!
     * \brief Used to check whether at least one message has been received on the handled topic.
     *
     * \return true if at least one message was received, otherwise false.
     */
    virtual bool hasMsg() const
    {
      assert(m_pimpl);
      return m_pimpl->hasMsg();
    };

    /*!
     * \brief Used to check whether at least one message has been received on the handled topic since the last call to getMsg().
     *
     * \return true if at least one message was received, otherwise false.
     */
    virtual bool newMsg() const
    {
      assert(m_pimpl);
      return m_pimpl->newMsg();
    };

    /*!
     * \brief Used to check whether getMsg() was called at least once on this SubscriberHandler.
     *
     * \return true if getMsg() was called at least once, otherwise false.
     */
    virtual bool usedMsg() const
    {
      assert(m_pimpl);
      return m_pimpl->usedMsg();
    };

    /*!
     * \brief Blocks until new data becomes available or until the timeout runs out or until a spurious wake-up.
     *
     * \param timeout    after this duration, this method will return a nullptr if no new data becomes available.
     * \return           the message if a new message is available after waking up, \p nullptr otherwise.
     */
    virtual typename MessageType::ConstSharedPtr waitForNew(const rclcpp::Duration& timeout)
    {
      assert(m_pimpl);
      return m_pimpl->waitForNew(timeout);
    };

    /*!
     * \brief Returns time of the last received message on the topic, handled by this SubscriberHandler.
     *
     * \return time when the last message was received.
     */
    virtual rclcpp::Time lastMsgTime() const
    {
      assert(m_pimpl);
      return m_pimpl->lastMsgTime();
    };

    /*!
     * \brief Returns the resolved (full) name of the topic, handled by this SubscriberHandler.
     *
     * \return name of the handled topic.
     */
    virtual std::string topicName() const
    {
      assert(m_pimpl);
      return m_pimpl->topicName();
    };

    /*!
     * \brief Returns the subscribed (unresolved) name of the topic, handled by this SubscriberHandler.
     *
     * \return name of the handled topic.
     */
    virtual std::string subscribedTopicName() const
    {
      assert(m_pimpl);
      return m_pimpl->m_topic_name;
    };

    /*!
     * \brief Returns number of publishers registered at the topic.
     *
     * \return number of publishers.
     */
    virtual uint32_t getNumPublishers() const
    {
      assert(m_pimpl);
      return m_pimpl->getNumPublishers();
    };

    /*!
     * \brief Sets the timeout for no received message.
     *
     * \param timeout    The new timeout for no received messages.
     */
    virtual void setNoMessageTimeout(const rclcpp::Duration& timeout)
    {
      assert(m_pimpl);
      return m_pimpl->setNoMessageTimeout(timeout);
    };

    /*!
     * \brief Enables the callbacks for the handled topic.
     *
     * If the SubscriberHandler object is stopped using the stop() method, no callbacks will be called
     * until the start() method is called.
     */
    virtual void start()
    {
      assert(m_pimpl);
      return m_pimpl->start();
    };

    /*!
     * \brief Disables the callbacks for the handled topic.
     *
     * All messages after this method is called will be ignored until start() is called again.
     * Timeout checking will also be disabled.
     */
    virtual void stop()
    {
      assert(m_pimpl);
      return m_pimpl->stop();
    };

  public:
    /*!
     * \brief Default constructor to avoid having to use pointers.
     *
     * It does nothing and the SubscriberHandler it constructs will also do nothing.
     * Use some of the other constructors for actual construction of a usable SubscriberHandler.
     */
    SubscriberHandler(){};

    /*!
     * \brief Main constructor.
     *
     * \param options    The common options struct (see documentation of SubscriberHandlerOptions).
     * \param topic_name Name of the topic to be handled by this subscribed.
     * \param args       Remaining arguments to be parsed (see other constructors).
     *
     */
    template <class... Types>
    SubscriberHandler(const SubscriberHandlerOptions& options, const std::string& topic_name, Types... args)
        : SubscriberHandler(
              [options, topic_name]() {
                SubscriberHandlerOptions opts = options;
                opts.topic_name = topic_name;
                return opts;
              }(),
              args...)
    {
    }

    /*!
     * \brief Convenience constructor overload.
     *
     * \param options          The common options struct (see documentation of SubscriberHandlerOptions).
     * \param message_callback The callback function to call when a new message is received (you can leave this argument empty and just use the
     * newMsg()/hasMsg() and getMsg() interface).
     *
     */
    SubscriberHandler(const SubscriberHandlerOptions& options, const message_callback_t& message_callback = {})
    {
      if (options.threadsafe)
      {
        m_pimpl = std::make_unique<ImplThreadsafe>(options, message_callback);
      } else
      {
        m_pimpl = std::make_unique<Impl>(options, message_callback);
      }
      if (options.autostart)
        start();
    };

    /*!
     * \brief Convenience constructor overload.
     *
     * \param options          The common options struct (see documentation of SubscriberHandlerOptions).
     * \param timeout_callback The callback function to call when a new message is not received for the duration, specified in \p options or in the \p
     * no_message_timeout parameter. \param args             Remaining arguments to be parsed (see other constructors).
     *
     */
    template <class... Types>
    SubscriberHandler(const SubscriberHandlerOptions& options, const timeout_callback_t& timeout_callback, Types... args)
        : SubscriberHandler(
              [options, timeout_callback]() {
                SubscriberHandlerOptions opts = options;
                opts.timeout_callback = timeout_callback;
                return opts;
              }(),
              args...)
    {
    }

    /*!
     * \brief Convenience constructor overload.
     *
     * \param options          The common options struct (see documentation of SubscriberHandlerOptions).
     * \param timeout_callback The callback method to call when a new message is not received for the duration, specified in \p options or in the \p
     * no_message_timeout parameter. \param obj1             The object on which the callback method \p timeout_callback will be called. \param args Remaining
     * arguments to be parsed (see other constructors).
     *
     */
    template <class ObjectType1, class... Types>
    SubscriberHandler(const SubscriberHandlerOptions& options,
                      void (ObjectType1::*const timeout_callback)(const std::string& topic_name, const rclcpp::Time& last_msg), ObjectType1* const obj1,
                      Types... args)
        : SubscriberHandler(
              [options, timeout_callback, obj1]() {
                SubscriberHandlerOptions opts = options;
                opts.timeout_callback =
                    timeout_callback == nullptr ? timeout_callback_t() : std::bind(timeout_callback, obj1, std::placeholders::_1, std::placeholders::_2);
                return opts;
              }(),
              args...)
    {
    }

    /*!
     * \brief Convenience constructor overload.
     *
     * \param options          The common options struct (see documentation of SubscriberHandlerOptions).
     * \param message_callback The callback method to call when a new message is received.
     * \param obj2             The object on which the callback method \p timeout_callback will be called.
     * \param args             Remaining arguments to be parsed (see other constructors).
     *
     */
    template <class ObjectType2, class... Types>
    SubscriberHandler(const SubscriberHandlerOptions& options, void (ObjectType2::*const message_callback)(typename MessageType::ConstSharedPtr),
                      ObjectType2* const obj2, Types... args)
        : SubscriberHandler(options, message_callback == nullptr ? message_callback_t() : std::bind(message_callback, obj2, std::placeholders::_1), args...)
    {
    }

    /*!
     * \brief Convenience constructor overload.
     *
     * \param options          The common options struct (see documentation of SubscriberHandlerOptions).
     * \param message_callback The callback method to call when a new message is received.
     * \param obj2             The object on which the callback method \p timeout_callback will be called.
     * \param timeout_callback The callback method to call when a new message is not received for the duration, specified in \p options or in the \p
     * no_message_timeout parameter. \param obj1             The object on which the callback method \p timeout_callback will be called. \param args Remaining
     * arguments to be parsed (see other constructors).
     *
     */
    template <class ObjectType1, class ObjectType2, class... Types>
    SubscriberHandler(const SubscriberHandlerOptions& options, void (ObjectType2::*const message_callback)(typename MessageType::ConstSharedPtr),
                      ObjectType2* const obj2, void (ObjectType1::*const timeout_callback)(const std::string& topic_name, const rclcpp::Time& last_msg),
                      ObjectType1* const obj1, Types... args)
        : SubscriberHandler(
              [options, timeout_callback, obj1]() {
                SubscriberHandlerOptions opts = options;
                opts.timeout_callback =
                    timeout_callback == nullptr ? timeout_callback_t() : std::bind(timeout_callback, obj1, std::placeholders::_1, std::placeholders::_2);
                return opts;
              }(),
              message_callback == nullptr ? message_callback_t() : std::bind(message_callback, obj2, std::placeholders::_1), args...)
    {
    }

    /*!
     * \brief Convenience constructor overload.
     *
     * \param options            The common options struct (see documentation of SubscriberHandlerOptions).
     * \param no_message_timeout If no message is received for this duration, the timeout callback function is called. If no timeout callback is specified, an
     * error message is printed to the console. \param args               Remaining arguments to be parsed (see other constructors).
     *
     */
    template <class... Types>
    SubscriberHandler(const SubscriberHandlerOptions& options, const rclcpp::Duration& no_message_timeout, Types... args)
        : SubscriberHandler(
              [options, no_message_timeout]() {
                SubscriberHandlerOptions opts = options;
                opts.no_message_timeout = no_message_timeout;
                return opts;
              }(),
              args...)
    {
    }

    /*!
     * \brief Convenience constructor overload.
     *
     * \param options          The common options struct (see documentation of SubscriberHandlerOptions).
     * \param timeout_manager  The manager for timeout callbacks.
     * \param args             Remaining arguments to be parsed (see other constructors).
     *
     */
    template <class... Types>
    SubscriberHandler(const SubscriberHandlerOptions& options, std::shared_ptr<mrs_lib::TimeoutManager>& timeout_manager, Types... args)
        : SubscriberHandler(options, timeout_manager, args...)
    {
    }

    ~SubscriberHandler() = default;
    // delete copy constructor and assignment operator (forbid copying shandlers)
    SubscriberHandler(const SubscriberHandler&) = delete;
    SubscriberHandler& operator=(const SubscriberHandler&) = delete;
    // define only move constructor and assignemnt operator
    SubscriberHandler(SubscriberHandler&& other)
    {
      this->m_pimpl = std::move(other.m_pimpl);
      other.m_pimpl = nullptr;
    }
    SubscriberHandler& operator=(SubscriberHandler&& other)
    {
      this->m_pimpl = std::move(other.m_pimpl);
      other.m_pimpl = nullptr;
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
  template <typename SubscriberHandler>
  using message_type = typename SubscriberHandler::message_type;

  /*!
   * \brief Helper function for object construstion e.g. in case of member objects.
   * This function is useful to avoid specifying object template parameters twice - once in definition of the variable and second time during object
   * construction. This function can deduce the template parameters from the type of the already defined object, because it returns the newly constructed object
   * as a reference argument and not as a return type.
   *
   * \param object The object to be constructed.
   * \param args   These arguments are passed to the object constructor.
   */
  template <typename Class, class... Types>
  void construct_object(Class& object, Types... args)
  {
    object = Class(args...);
  }
}  // namespace mrs_lib

#ifndef SUBSCRIBER_HANDLER_HPP
#include <mrs_lib/impl/subscriber_handler.hpp>
#endif  // SUBSCRIBER_HANDLER_HPP

#endif  // SUBRSCRIBER_HANDLER_H
