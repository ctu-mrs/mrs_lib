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
#include <boost/circular_buffer.hpp>

#include <nav_msgs/Odometry.h>

namespace mrs_lib
{
  static const ros::Duration no_timeout = ros::Duration(0);
}

#include <impl/subscribe_handler.hpp>

namespace mrs_lib
{
  /* SubscribeHandler class //{ */
  /**
  * \brief Base class for pointers, returned by the SubscribeMgr.
  *
  * This class should not be manually instantiated by the user.
  * Instead, use this class to store smart pointers returned by the SubscribeMgr.
  *
  */
  template <typename MessageType>
  class SubscribeHandler : public impl::SubscribeHandler_base
  {
    public:
    /*!
      * \brief Type for the timeout callback function. For clarity and consistency, it is recommended to use the SubscribeMgr::timeout_callback_t instead.
      */
      using timeout_callback_t = impl::SubscribeHandler_base::timeout_callback_t;

    public:
    /*!
      * \brief Returns the last received message on the topic, handled by this SubscribeHandler.
      *
      * \return The last received message.
      */
      virtual MessageType get_data() = 0;

    protected:
    /*!
      * \brief Main constructor.
      *
      * \param nh                     The topics will be subscribed using this node handle.
      * \param topic_name             Name of the topic this SubscribeHandler will subscribe to and handle.
      * \param node_name              Optional node name used when printing warnings and errors.
      * \param no_message_timeout     After this duration, the timeout_callback will be called. If set to zero, this functionality is disabled.
      * \param timeout_callback       This function is called if no messages are received for no_message_timeout.
      */
      SubscribeHandler(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          const std::string& node_name,
          const ros::Duration& no_message_timeout,
          const timeout_callback_t& timeout_callback
        )
        : impl::SubscribeHandler_base(
            nh,
            topic_name,
            node_name,
            no_message_timeout,
            timeout_callback
          )
      {};
  };
  //}

  /* /1* SubscribeBuffer class //{ *1/ */
  /* template <typename MessageWithHeaderType> */
  /* class SubscribeBuffer : public impl::SubscribeHandler_impl<MessageWithHeaderType> */
  /* { */
  /*   private: */
  /*     using impl_class_t = impl::SubscribeHandler_impl<MessageWithHeaderType>; */

  /*   public: */
  /*     using buffer_t = typename boost::circular_buffer<MessageWithHeaderType>; */
  /*     using iterator_t = typename buffer_t::iterator; */
  /*     SubscribeBuffer( */
  /*         ros::NodeHandle& nh, */
  /*         const std::string& topic_name, */
  /*         uint32_t queue_size, */
  /*         uint32_t buffer_size, */
  /*         const ros::TransportHints& transport_hints = ros::TransportHints(), */
  /*         const ros::Duration& no_message_timeout = mrs_lib::no_timeout, */
  /*         const std::string& node_name = std::string() */
  /*       ) */
  /*       : impl_class_t( */
  /*           nh, */
  /*           topic_name, */
  /*           queue_size, */
  /*           transport_hints, */
  /*           no_message_timeout, */
  /*           node_name */
  /*         ), */
  /*         m_bfr(buffer_size) */
  /*     {}; */

  /*     /1* get_data() method //{ *1/ */
  /*     virtual MessageWithHeaderType get_data() */
  /*     { */
  /*       std::lock_guard<std::mutex> lck(m_mtx); */
  /*       return impl_class_t::get_data(); */
  /*     } */
  /*     //} */

  /*     /1* get_closest() method //{ *1/ */
  /*     // Finds the closest message in the buffer to the time stamp. */
  /*     // returns -1 if requested message would be newer than newest message in buffer */
  /*     // returns 1 if requested message would be older than oldest message in buffer */
  /*     // returns 0 otherwise */
  /*     int get_closest(ros::Time stamp, MessageWithHeaderType& closest_out) */
  /*     { */
  /*       std::lock_guard<std::mutex> lck(m_mtx); */
  /*       if (m_bfr.empty()) */
  /*         return -1; */
  /*       iterator_t cl_it; */
  /*       const int ret = get_closest_impl(stamp, cl_it); */
  /*       closest_out = *cl_it; */
  /*       return ret; */
  /*     } */
  /*     //} */

  /*     /1* get_next_closest() method //{ *1/ */
  /*     // Finds the first message after the closest message in the buffer to the time stamp */
  /*     // (for example for interpolation). */
  /*     // returns -1 if requested message would be newer than newest message in buffer */
  /*     // returns 1 if requested message would be older than oldest message in buffer */
  /*     // returns 0 otherwise */
  /*     int get_next_closest(ros::Time stamp, MessageWithHeaderType& next_out) */
  /*     { */
  /*       std::lock_guard<std::mutex> lck(m_mtx); */
  /*       iterator_t cl_it; */
  /*       int ret = get_closest_impl(stamp, cl_it); */
  /*       if (ret >= 0) */
  /*       { */
  /*         iterator_t nx_it = cl_it; */
  /*         nx_it++; */
  /*         if (nx_it == m_bfr.end()) */
  /*         { */
  /*           next_out = *cl_it; */
  /*           ret = -1; */
  /*         } else */
  /*         { */
  /*           next_out = *nx_it; */
  /*         } */
  /*       } */
  /*       return ret; */
  /*     } */
  /*     //} */

  /*   protected: */
  /*     mutable std::mutex m_mtx; */
  /*     buffer_t m_bfr; */
  /*     /1* const size_t m_bfr_max; *1/ */

  /*   protected: */
  /*     /1* data_callback() method //{ *1/ */
  /*     virtual void data_callback(const MessageWithHeaderType& msg) */
  /*     { */
  /*       std::lock_guard<std::mutex> lck(m_mtx); */
      
  /*       return data_callback_impl(msg); */
  /*     } */
  /*     //} */

  /*   private: */
  /*     /1* get_closest_impl() method //{ *1/ */
  /*     int get_closest_impl(ros::Time stamp, iterator_t& closest_it) */
  /*     { */
  /*       // if no message in the buffer is newer */
  /*       // than the requested stamp, the user is requesting */
  /*       // a too new message */
  /*       int result = -1; */
  /*       bool first_elem = true; */
  /*       closest_it = m_bfr.begin(); */
      
  /*       for (iterator_t it = m_bfr.begin(); it != m_bfr.end(); it++) */
  /*       { */
  /*         const MessageWithHeaderType& msg = *it; */
  /*         const double cur_diff = (impl_class_t::get_header(msg).stamp - stamp).toSec(); */
      
  /*         if (cur_diff >= 0) */
  /*         { // take advantage of the list being inherently sorted */
  /*           /1* closest_out = msg; *1/ */
  /*           closest_it = it; */
  /*           // if this is the oldest element in the buffer */
  /*           // and the stamp is older, the user is requesting */
  /*           // a too old message */
  /*           if (first_elem) */
  /*             result = 1; */
  /*           else */
  /*             result = 0; */
  /*           break; */
  /*         } */

  /*         first_elem = false; */
  /*       } */
  /*       return result; */
  /*     } */
  /*     //} */

  /*   protected: */
  /*     /1* data_callback_impl() method for messages//{ *1/ */
  /*     virtual void data_callback_impl(const MessageWithHeaderType& msg) */
  /*     { */
  /*       ros::Time now = ros::Time::now(); */ 
  /*       const bool time_reset = impl_class_t::check_time_reset(now); */
  /*       const bool message_valid = impl_class_t::check_message_valid(msg); */
  /*       if (message_valid || time_reset) */
  /*       { */
  /*         if (time_reset) */
  /*         { */
  /*           ROS_WARN("[%s]: Detected jump back in time of %f. Resetting time consistency checks and clearing message buffer.", impl::SubscribeHandler_base::m_node_name.c_str(), (impl::SubscribeHandler_base::m_last_msg_received - now).toSec()); */
  /*           m_bfr.clear(); */
  /*         } */
  /*         impl_class_t::data_callback_unchecked(msg, now); */
  /*         m_bfr.push_back(msg); */
  /*       } else */
  /*       { */
  /*         ROS_WARN("[%s]: New message from topic '%s' is older than the latest message, skipping it.", impl::SubscribeHandler_base::m_node_name.c_str(), impl::SubscribeHandler_base::resolved_topic_name().c_str()); */
  /*       } */
  /*     } */
  /*     //} */

  /* }; */

  /* //} */

/*!
  * \brief Helper typedef for a more convenient usage of the shared_ptrs for the SubscribeHandler.
  */
  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType> >;

  /* SubscriberMgr class //{ */
  /**
  * \brief Manager class for creating SunscribeHandler objects.
  *
  * This class automatically checks for subscribe errors when creating SubscribeHandlers.
  * Use the create_handler() method to create new SubscribeHandler objects.
  * Use the loaded_successfully() method to check if all subscribes have been successful so far.
  *
  */
  class SubscribeMgr
  {
    public:
      using timeout_callback_t = impl::SubscribeHandler_base::timeout_callback_t;
      template <typename MessageType>
      using message_callback_t = typename impl::SubscribeHandler_impl<MessageType>::message_callback_t;

    public:
    /*!
      * \brief Main constructor.
      *
      * \param nh                     The topics will be subscribed using this node handle.
      * \param node_name              Optional node name used when printing warnings and errors.
      */
      SubscribeMgr(ros::NodeHandle& nh, std::string node_name = std::string()) : m_nh(nh), m_node_name(node_name), m_load_successful(true) {};

      /* create_handler() method //{ */
    /*!
      * \brief Instantiates a new SubscribeHandler object.
      *
      * \param nh                     The topics will be subscribed using this node handle.
      * \param node_name              Optional node name used when printing warnings and errors.
      */
      template <typename MessageType, bool time_consistent=false>
      SubscribeHandlerPtr<MessageType> create_handler(
            const std::string& topic_name,
            const ros::Duration& no_message_timeout = mrs_lib::no_timeout,
            const timeout_callback_t& timeout_callback = timeout_callback_t(),
            const message_callback_t<MessageType>& message_callback = message_callback_t<MessageType>(),
            const bool threadsafe = true,
            uint32_t queue_size = 1,
            const ros::TransportHints& transport_hints = ros::TransportHints()
          )
      {
        SubscribeHandlerPtr<MessageType> ptr;
        if (threadsafe)
        {
          auto local_ptr = std::make_shared<impl::SubscribeHandler_impl<MessageType, time_consistent> >
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
          // Important! Otherwise the message callback will crash when trying to pass pointer to self.
          local_ptr->set_ptr(local_ptr);
          ptr = local_ptr;
        } else
        {
          auto local_ptr = std::make_shared<impl::SubscribeHandler_threadsafe<MessageType, time_consistent> >
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
          // Important! Otherwise the message callback will crash when trying to pass pointer to self.
          local_ptr->set_ptr(local_ptr);
          ptr = local_ptr;
        }
        m_load_successful = m_load_successful && ptr->ok();
        return ptr;
      }
      //}

      /* /1* create_buffer() method //{ *1/ */
      /* template <typename MessageWithHeaderType> */
      /* SubscribeBufferPtr<MessageWithHeaderType> create_buffer( */
      /*       const std::string& topic_name, */
      /*       uint32_t buffer_size, */
      /*       const ros::Duration& no_message_timeout = mrs_lib::no_timeout, */
      /*       uint32_t queue_size = 1, */
      /*       const ros::TransportHints& transport_hints = ros::TransportHints() */
      /*     ) */
      /* { */
      /*   SubscribeBufferPtr<MessageWithHeaderType> ptr = std::make_shared<SubscribeBuffer<MessageWithHeaderType> > */
      /*     ( */
      /*       m_nh, */
      /*       topic_name, */
      /*       queue_size, */
      /*       buffer_size, */
      /*       transport_hints, */
      /*       no_message_timeout, */
      /*       m_node_name */
      /*     ); */
      /*   m_load_successful = m_load_successful && ptr->ok(); */
      /*   return ptr; */
      /* } */
      /* //} */

      bool loaded_successfully()
      {
        return m_load_successful;
      }

    private:
      ros::NodeHandle m_nh;
      std::string m_node_name;
      bool m_load_successful;

  };
  //}

}

#endif // SUBRSCRIBE_HANDLER_H
