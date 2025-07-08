// clang: TomasFormat
/**  \file
     \brief Defines PublisherHandler and related convenience classes for upgrading the ROS publisher
     \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef PUBLISHER_HANDLER_H
#define PUBLISHER_HANDLER_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time_source.hpp>

#include <atomic>
#include <string>
#include <mutex>

namespace mrs_lib
{

struct PublisherHandlerOptions
{
  PublisherHandlerOptions(const rclcpp::Node::SharedPtr& node) : node(node) {
  }

  PublisherHandlerOptions() = default;

  rclcpp::Node::SharedPtr node;

  /**
   * @brief when > 0, the publisher output is limited to this rate in [Hz]
   */
  double throttle_rate = 0;

  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
};

/* class PublisherHandler_impl //{ */

/**
 * @brief implementation of the publisher handler
 */
template <class TopicType>
class PublisherHandler_impl {

public:
  /**
   * @brief default constructor
   */
  PublisherHandler_impl(void);

  /**
   * @brief default destructor
   */
  ~PublisherHandler_impl(void){};

  /**
   * @brief full constructor with options
   *
   * @param options subscribe handler options
   * @param address topic address
   */
  PublisherHandler_impl(const PublisherHandlerOptions& options, const std::string& address);

  /**
   * @brief publish message
   *
   * @param msg data
   *
   */
  void publish(const TopicType& msg);

  /**
   * @brief publish message, boost ptr overload
   *
   * @param msg message
   */
  void publish(const std::shared_ptr<TopicType>& msg);

  /**
   * @brief publish message, boost const ptr overload
   *
   * @param msg message
   */
  /* void publish(const std::shared_ptr<TopicType const>& msg); */

  /**
   * @brief publish message, boost const ptr overload
   *
   * @param msg message
   */
  void publish(typename TopicType::ConstSharedPtr msg);

  /**
   * @brief get number of subscribers
   *
   * @return the number of subscribers
   */
  unsigned int getNumSubscribers(void);

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<rclcpp::Publisher<TopicType>> publisher_;

  std::mutex mutex_publisher_;

  std::atomic<bool> publisher_initialized_;

  std::string address_;

  bool   throttle_        = false;
  double throttle_min_dt_ = 0;

  rclcpp::Time last_time_published_;
};

//}

/* class PublisherHandler //{ */

/**
 * @brief user wrapper of the publisher handler implementation
 */
template <class TopicType>
class PublisherHandler {

public:
  /**
   * @brief generic constructor
   */
  PublisherHandler(void){};

  /**
   * @brief generic destructor
   */
  ~PublisherHandler(void){};

  /**
   * @brief operator=
   *
   * @param other
   *
   * @return
   */
  PublisherHandler& operator=(const PublisherHandler& other);

  /**
   * @brief copy constructor
   *
   * @param other
   */
  PublisherHandler(const PublisherHandler& other);

  /**
   * @brief slim constructor
   *
   * @param node
   * @param address
   */
  PublisherHandler(const rclcpp::Node::SharedPtr& node, const std::string& address);

  /**
   * @brief full constructor
   *
   * @param options
   * @param address
   */
  PublisherHandler(const PublisherHandlerOptions& options, const std::string& address);

  /**
   * @brief publish message
   *
   * @param msg
   */
  void publish(const TopicType& msg);

  /**
   * @brief publish message, std ptr overload
   *
   * @param msg
   */
  void publish(const std::shared_ptr<TopicType>& msg);

  /**
   * @brief publish message, std const ptr overload
   *
   * @param msg
   */
  /* void publish(const std::shared_ptr<TopicType const>& msg); */

  /**
   * @brief publish message, std const ptr overload
   *
   * @param msg
   */
  void publish(typename TopicType::ConstSharedPtr msg);

  /**
   * @brief get number of subscribers
   *
   * @return the number of subscribers
   */
  unsigned int getNumSubscribers(void);

private:
  std::shared_ptr<PublisherHandler_impl<TopicType>> impl_;
};

//}

}  // namespace mrs_lib

#ifndef PUBLISHER_HANDLER_HPP
#include <mrs_lib/impl/publisher_handler.hpp>
#endif  // PUBLISHER_HANDLER_H

#endif  // PUBLISHER_HANDLER_H
