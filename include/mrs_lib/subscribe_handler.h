// clang: MatousFormat
/**  \file
     \brief Defines SubscribeMgr and related convenience classes for subscribing ROS topics.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef SUBRSCRIBE_HANDLER_H
#define SUBRSCRIBE_HANDLER_H

#include <ros/ros.h>
#include <string>
#include <mutex>
/* #include <boost/circular_buffer.hpp> */

#include <nav_msgs/Odometry.h>

namespace mrs_lib
{
  static const ros::Duration no_timeout = ros::Duration(0);

  class SubscribeMgr;

  template <typename MessageType>
  class SubscribeHandler;

/*!
  * \brief Helper typedef for a more convenient usage of the shared_ptrs for the SubscribeHandler.
  */
  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType>>;

/*!
  * \brief Helper alias for convenient extraction of handled message type from a SubscribeHandlerPtr.
  */
  template<typename SubscribeHandlerPtr>
  using message_type = typename std::remove_const<typename SubscribeHandlerPtr::element_type::message_type>::type;
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
      using message_callback_t = std::function<void(SubscribeHandlerPtr<MessageType>)>;

    /*!
      * \brief SubscribeMgr has to be friend to enable proper construction of this class.
      */
      friend class SubscribeMgr;

    public:
    /*!
      * \brief Returns the last received message on the topic, handled by this SubscribeHandler.
      *
      * \return the last received message.
      */
      typename MessageType::ConstPtr get_data() {assert(m_pimpl); return m_pimpl->get_data();};

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
    /*!
      * \brief Main constructor.
      *
      * \warning Do not use this constructor to instantiate this class! Use the SubscribeMgr::create_handler() method instead!
      */
      SubscribeHandler() {};

    private:
      void set_impl_ptr(std::unique_ptr<impl::SubscribeHandler_impl<MessageType>>&& pimpl) {m_pimpl = std::move(pimpl);};
      std::unique_ptr<impl::SubscribeHandler_impl<MessageType>> m_pimpl;
  };
  //}

  /* SubscribeMgr class //{ */
  /**
  * \brief Manager class for creating SubscribeHandler objects.
  *
  * This class serves to instantiate objects of the SubscribeHandler.
  * Use the create_handler() method to create new SubscribeHandler objects.
  *
  * Example usage:
  * \include src/subscribe_handler/example.cpp
  *
  */
  class SubscribeMgr
  {
    public:
    /*!
      * \brief Convenience type for the timeout callback function.
      */
      using timeout_callback_t = std::function<void(const std::string&, const ros::Time&, const int)>;
    /*!
      * \brief Convenience type for the message callback function.
      */
      template <typename MessageType>
      using message_callback_t = typename SubscribeHandler<MessageType>::message_callback_t;

    public:
    /*!
      * \brief Main constructor.
      *
      * \param nh                     The topics will be subscribed using this node handle.
      * \param node_name              Optional node name used when printing warnings and errors.
      */
      SubscribeMgr(ros::NodeHandle& nh, std::string node_name = std::string()) : m_nh(nh), m_node_name(node_name) {};

      /* create_handler() method //{ */
    /*!
      * \brief Instantiates a new SubscribeHandler object.
      *
      * \tparam time_consistent     Whether the handler should discard messages with time stamp earlier than the latest message (enforce time consistency of messages).
      *
      * \param topic_name           Name of the topic the new object will be handling (subscribe to).
      * \param no_message_timeout   After this duration has passed without receiving any new messages on the handled topic, the \p timeout_callback will be called.
      * \param timeout_callback     The method to call if no messages have arrived for \p no_message_timeout. If empty a throttled ROS warning will be prited instead of calling \p timeout_callback.
      * \param message_callback     Optional callback when receiving a new message.
      * \param threadsafe           Whether the handler should be mutexed.
      * \param autostart            Whether the handler should be automatically started after construction (callbacks will be enabled immediately).
      * \param queue_size           Will be passed to the ROS NodeHandle when subscribing (see ROS docs for explanation).
      * \param transport_hints      Will be passed to the ROS NodeHandle when subscribing (see ROS docs for explanation).
      *
      * \returns                    std::shared_ptr to the new SubscribeHandler object. When the object is destroyed, the callbacks will not be called anymore.
      */
      template <typename MessageType, bool time_consistent=false>
      SubscribeHandlerPtr<MessageType> create_handler(
            const std::string& topic_name,
            const ros::Duration& no_message_timeout = mrs_lib::no_timeout,
            const timeout_callback_t& timeout_callback = timeout_callback_t(),
            const message_callback_t<MessageType>& message_callback = message_callback_t<MessageType>(),
            const bool threadsafe = true,
            const bool autostart = true,
            const uint32_t queue_size = 10,
            const ros::TransportHints& transport_hints = ros::TransportHints()
          )
      {
        SubscribeHandlerPtr<MessageType> ptr;
        if (threadsafe)
        {
          using impl_t = impl::SubscribeHandler_threadsafe<MessageType>;
          auto impl_ptr = std::make_unique<impl_t>
            (
              m_nh,
              topic_name,
              m_node_name,
              message_callback,
              no_message_timeout,
              timeout_callback,
              queue_size,
              transport_hints
            );
          impl_ptr->template set_data_callback<time_consistent>();
          ptr = std::make_shared<SubscribeHandler<MessageType>>();
          // Important! Otherwise the message callback will crash when trying to pass pointer to self.
          impl_ptr->set_owner_ptr(ptr);
          // Also important! Otherwise there will be nullptr dereferencing.
          ptr->set_impl_ptr(std::move(impl_ptr));
        } else
        {
          using impl_t = impl::SubscribeHandler_impl<MessageType>;
          auto impl_ptr = std::make_unique<impl_t>
            (
              m_nh,
              topic_name,
              m_node_name,
              message_callback,
              no_message_timeout,
              timeout_callback,
              queue_size,
              transport_hints
            );
          impl_ptr->template set_data_callback<time_consistent>();
          ptr = std::make_shared<SubscribeHandler<MessageType>>();
          // Important! Otherwise the message callback will crash when trying to pass pointer to self.
          impl_ptr->set_owner_ptr(ptr);
          // Also important! Otherwise there will be nullptr dereferencing.
          ptr->set_impl_ptr(std::move(impl_ptr));
        }
        if (autostart)
          ptr->start();
        return ptr;
      }
      //}

    private:
      ros::NodeHandle m_nh;
      std::string m_node_name;

  };
  //}

}

#endif // SUBRSCRIBE_HANDLER_H
