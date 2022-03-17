/**  \file
     \brief Defines PublisherHandler and related convenience classes for upgrading the ROS publisher
     \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef PUBLISHER_HANDLER_H
#define PUBLISHER_HANDLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <mutex>

namespace mrs_lib
{

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
   * @brief constructor
   *
   * @param nh ROS node handler
   * @param address topic address
   * @param buffer_size buffer size
   * @param latch latching
   */
  PublisherHandler_impl(ros::NodeHandle& nh, const std::string& address, const unsigned int& buffer_size = 1, const bool& latch = false,
                        const double& rate = 0.0);

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
   * @param msg
   */
  void publish(const boost::shared_ptr<TopicType>& msg);

  /**
   * @brief publish message, boost const ptr overload
   *
   * @param msg
   */
  void publish(const boost::shared_ptr<TopicType const>& msg);

  /**
   * @brief get number of subscribers
   *
   * @return the number of subscribers
   */
  unsigned int getNumSubscribers(void);

private:
  ros::Publisher    publisher_;
  std::mutex        mutex_publisher_;
  std::atomic<bool> publisher_initialized_;

  bool      throttle_ = false;
  double    throttle_min_dt_;
  ros::Time last_time_published_;
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
   * @brief constructor
   *
   * @param nh ROS node handler
   * @param address topic address
   * @param buffer_size buffer size
   * @param latch latching
   */
  PublisherHandler(ros::NodeHandle& nh, const std::string& address, const unsigned int& buffer_size = 1, const bool& latch = false, const double& rate = 0);

  /**
   * @brief publish message
   *
   * @param msg
   */
  void publish(const TopicType& msg);

  /**
   * @brief publish message, boost ptr overload
   *
   * @param msg
   */
  void publish(const boost::shared_ptr<TopicType>& msg);

  /**
   * @brief publish message, boost const ptr overload
   *
   * @param msg
   */
  void publish(const boost::shared_ptr<TopicType const>& msg);

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

#include <impl/publisher_handler.hpp>

#endif  // PUBLISHER_HANDLER_H
