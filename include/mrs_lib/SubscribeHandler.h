#ifndef SUBRSCRIBEHANDLER_H
#define SUBRSCRIBEHANDLER_H

#include <ros/ros.h>
#include <string>
#include <mutex>
#include <boost/circular_buffer.hpp>

#include <nav_msgs/Odometry.h>

namespace mrs_lib
{
  static const ros::Duration no_timeout = ros::Duration(0);
}

#include <impl/SubscribeHandler_impl.h>

namespace mrs_lib
{
  /* SubscribeHandler class //{ */
  // adds the get_data templated method - this is the class the user should use
  // for pointers etc.
  template <typename MessageType>
  class SubscribeHandler : public impl::SubscribeHandler_base
  {
    public:
      virtual MessageType get_data() = 0;

    protected:
      SubscribeHandler(
          ros::NodeHandle& nh,
          ros::Duration no_message_timeout,
          const std::string& topic_name,
          const std::string& node_name
        )
        : impl::SubscribeHandler_base(
            nh,
            no_message_timeout,
            topic_name,
            node_name
          )
      {};
  };
  //}

  /* SubscribeBuffer class //{ */
  template <typename MessageWithHeaderType>
  class SubscribeBuffer : public impl::SubscribeHandler_impl<MessageWithHeaderType>
  {
    private:
      using impl_class_t = impl::SubscribeHandler_impl<MessageWithHeaderType>;

    public:
      using buffer_t = typename boost::circular_buffer<MessageWithHeaderType>;
      using iterator_t = typename buffer_t::iterator;
      SubscribeBuffer(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          uint32_t buffer_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = mrs_lib::no_timeout,
          const std::string& node_name = std::string()
        )
        : impl_class_t(
            nh,
            topic_name,
            queue_size,
            transport_hints,
            no_message_timeout,
            node_name
          ),
          m_bfr(buffer_size)
      {};

      /* get_data() method //{ */
      virtual MessageWithHeaderType get_data()
      {
        std::lock_guard<std::mutex> lck(m_mtx);
        return impl_class_t::get_data();
      }
      //}

      /* get_closest() method //{ */
      // Finds the closest message in the buffer to the time stamp.
      // returns -1 if requested message would be newer than newest message in buffer
      // returns 1 if requested message would be older than oldest message in buffer
      // returns 0 otherwise
      int get_closest(ros::Time stamp, MessageWithHeaderType& closest_out)
      {
        std::lock_guard<std::mutex> lck(m_mtx);
        iterator_t cl_it;
        const int ret = get_closest_impl(stamp, cl_it);
        if (ret >= 0)
          closest_out = *cl_it;
        return ret;
      }
      //}

      /* get_next_closest() method //{ */
      // Finds the first message after the closest message in the buffer to the time stamp.
      // returns -1 if requested message would be newer than newest message in buffer
      // returns 1 if requested message would be older than oldest message in buffer
      // returns 0 otherwise
      int get_next_closest(ros::Time stamp, MessageWithHeaderType& next_out)
      {
        std::lock_guard<std::mutex> lck(m_mtx);
        iterator_t cl_it;
        int ret = get_closest_impl(stamp, cl_it);
        if (ret >= 0)
        {
          iterator_t nx_it = cl_it;
          nx_it++;
          if (nx_it == m_bfr.end())
          {
            next_out = *cl_it;
            ret = -1;
          } else
          {
            next_out = *nx_it;
          }
        }
        return ret;
      }
      //}

    protected:
      mutable std::mutex m_mtx;
      buffer_t m_bfr;
      /* const size_t m_bfr_max; */

    protected:
      /* data_callback() method //{ */
      virtual void data_callback(const MessageWithHeaderType& msg)
      {
        std::lock_guard<std::mutex> lck(m_mtx);
      
        return data_callback_impl(msg);
      }
      //}

    private:
      /* get_closest_impl() method //{ */
      int get_closest_impl(ros::Time stamp, iterator_t& closest_it)
      {
        // if no message in the buffer is newer
        // than the requested stamp, the user is requesting
        // a too new message
        int result = -1;
        bool first_elem = true;
        closest_it = m_bfr.end();
      
        for (iterator_t it = m_bfr.begin(); it != m_bfr.end(); it++)
        {
          const MessageWithHeaderType& msg = *it;
          const double cur_diff = (impl_class_t::get_header(msg).stamp - stamp).toSec();
      
          if (cur_diff >= 0)
          { // take advantage of the list being inherently sorted
            /* closest_out = msg; */
            closest_it = it;
            // if this is the oldest element in the buffer
            // and the stamp is older, the user is requesting
            // a too old message
            if (first_elem)
              result = 1;
            else
              result = 0;
            break;
          }

          first_elem = false;
        }
        return result;
      }
      //}

    protected:
      /* data_callback_impl() method for messages//{ */
      virtual void data_callback_impl(const MessageWithHeaderType& msg)
      {
        if (!m_bfr.empty()
         && impl_class_t::get_header(msg).stamp < impl_class_t::get_header(m_bfr.back()).stamp)
        {
          ROS_WARN("[%s]: New message from topic '%s' is older than the latest message in the buffer, skipping it.", impl::SubscribeHandler_base::m_node_name.c_str(), impl::SubscribeHandler_base::resolved_topic_name().c_str());
        } else
        {
          m_bfr.push_back(msg);
          impl_class_t::data_callback(msg);
        }
      }
      //}

  };

  //}

  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType> >;
  template <typename MessageWithHeaderType>
  using SubscribeBufferPtr = std::shared_ptr<SubscribeBuffer<MessageWithHeaderType> >;

  /* SubscriberMgr class //{ */
  class SubscribeMgr
  {
    public:
      SubscribeMgr(ros::NodeHandle& nh, std::string node_name = std::string()) : m_nh(nh), m_node_name(node_name), m_load_successful(true) {};

      /* create_handler() method //{ */
      template <typename MessageType, bool time_consistent=false>
      SubscribeHandlerPtr<MessageType> create_handler(
            const std::string& topic_name,
            uint32_t queue_size,
            const ros::TransportHints& transport_hints = ros::TransportHints(),
            ros::Duration no_message_timeout = mrs_lib::no_timeout
          )
      {
        SubscribeHandlerPtr<MessageType> ptr = std::make_shared<impl::SubscribeHandler_impl<MessageType, time_consistent> >
          (
            m_nh,
            topic_name,
            queue_size,
            transport_hints,
            no_message_timeout,
            m_node_name
          );
        m_load_successful = m_load_successful && ptr->ok();
        return ptr;
      }
      //}

      /* create_handler_threadsafe() method //{ */
      template <typename MessageType, bool time_consistent=false>
      SubscribeHandlerPtr<MessageType> create_handler_threadsafe(
            const std::string& topic_name,
            uint32_t queue_size,
            const ros::TransportHints& transport_hints = ros::TransportHints(),
            ros::Duration no_message_timeout = mrs_lib::no_timeout
          )
      {
        SubscribeHandlerPtr<MessageType> ptr = std::make_shared<impl::SubscribeHandler_threadsafe<MessageType, time_consistent> >
          (
            m_nh,
            topic_name,
            queue_size,
            transport_hints,
            no_message_timeout,
            m_node_name
          );
        m_load_successful = m_load_successful && ptr->ok();
        return ptr;
      }
      //}

      /* create_buffer() method //{ */
      template <typename MessageWithHeaderType>
      SubscribeBufferPtr<MessageWithHeaderType> create_buffer(
            const std::string& topic_name,
            uint32_t queue_size,
            uint32_t buffer_size,
            const ros::TransportHints& transport_hints = ros::TransportHints(),
            ros::Duration no_message_timeout = mrs_lib::no_timeout
          )
      {
        SubscribeBufferPtr<MessageWithHeaderType> ptr = std::make_shared<SubscribeBuffer<MessageWithHeaderType> >
          (
            m_nh,
            topic_name,
            queue_size,
            buffer_size,
            transport_hints,
            no_message_timeout,
            m_node_name
          );
        m_load_successful = m_load_successful && ptr->ok();
        return ptr;
      }
      //}

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

#endif // SUBRSCRIBEHANDLER_H
