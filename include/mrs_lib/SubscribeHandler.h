#ifndef SUBRSCRIBEHANDLER_H
#define SUBRSCRIBEHANDLER_H

#include <ros/ros.h>
#include <string>
#include <mutex>

namespace mrs_lib
{
  template <typename MessageType>
  class SubscribeHandler_base;
  template <typename MessageType>
  class SubscribeHandler_threadsafe;
  template <typename MessageType>
  class SubscribeHandler;
  template <typename MessageType>
  using SubscribeHandlerPtr = std::shared_ptr<SubscribeHandler<MessageType>>;

  template <typename MessageType>
  class SubscribeHandler
  {
    public:
      static SubscribeHandlerPtr<MessageType> create_handler(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler::no_timeout,
          const std::string& node_name = std::string()
          )
      {
        return std::make_shared<SubscribeHandler_base<MessageType> >(
          nh,
          topic_name,
          queue_size,
          transport_hints,
          no_message_timeout,
          node_name
          );
      }
  
      static SubscribeHandlerPtr<MessageType> create_handler_threadsafe(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler::no_timeout,
          const std::string& node_name = std::string()
          )
      {
        return std::make_shared<SubscribeHandler_threadsafe<MessageType> >(
          nh,
          topic_name,
          queue_size,
          transport_hints,
          no_message_timeout,
          node_name
          );
      }
  
    public:
      virtual bool ok() = 0;
      virtual bool has_data() = 0;
      virtual MessageType get_data(bool reset_data_flag = false) = 0;
      virtual MessageType get_data_reset() = 0;

    public:
      static const ros::Duration no_timeout;

  };

  template <typename MessageType>
  class SubscribeHandler_base : public SubscribeHandler<MessageType>
  {
    public:
      SubscribeHandler_base(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          );

      virtual bool ok();
      virtual bool has_data();
      virtual MessageType get_data(bool reset_data_flag = false);
      virtual MessageType get_data_reset();

    private:
      ros::Duration m_no_message_timeout;
      std::string m_node_name;
  
    private:
      ros::Subscriber m_sub;
      bool m_new_data_ready;
      MessageType m_latest_message;
      ros::Time m_last_msg_received;
      bool m_ok;

    protected:
      virtual void data_callback(const MessageType& msg);

  };

  template <typename MessageType>
  class SubscribeHandler_threadsafe : public SubscribeHandler_base<MessageType>
  {
    public:
      SubscribeHandler_threadsafe(
          ros::NodeHandle& nh,
          const std::string& topic_name,
          uint32_t queue_size,
          const ros::TransportHints& transport_hints = ros::TransportHints(),
          ros::Duration no_message_timeout = SubscribeHandler_base<MessageType>::no_timeout,
          const std::string& node_name = std::string()
          );

      virtual bool ok();
      virtual bool has_data();
      virtual MessageType get_data(bool reset_data_flag = false);

    protected:
      virtual void data_callback(const MessageType& msg);

    private:
      std::mutex m_mtx;
  };
}

#include "SubscribeHandler.tpp"

#endif // SUBRSCRIBEHANDLER_H
