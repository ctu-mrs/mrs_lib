// clang: MatousFormat
/**  \file
     \brief Defines DynamicPublisher for easy debug publishing of ROS messages.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Tomáš Báča  - bacatoma@fel.cvut.cz
 */

#ifndef DYNAMIC_PUBLISHER_H
#define DYNAMIC_PUBLISHER_H

#include <mutex>
#include <ros/ros.h>
#include <mrs_lib/publisher_handler.h>

namespace mrs_lib
{

  /**
  * \brief A helper class for easy publishing of ROS messages for debugging purposes.
  *
  * This class enables you to just call the publish() method with a topic name and a message without the need to advertise the topic.
  * 
  * \note This class should only be used for debugging and not for regular publishing as it introduces some overhead.
  *
  */
  class DynamicPublisher
  {
  public:
    /*!
      * \brief A no-parameter constructor.
      *
      * This overload will use a ros::NodeHandle with default arguments for advertising new topics, so remappings, namespaces etc. will be ignored.
      */
    DynamicPublisher();

    /*!
      * \brief The main constructor.
      *
      * This overload uses the ros::NodeHandle that you provided for advertising new topics.
      * The recommended way of constructing a DynamicPublisher.
      */
    DynamicPublisher(const ros::NodeHandle& nh);

    /*!
      * \brief Publishes a message to a topic, advertising the topic if necessary.
      *
      * The topic is advertised with the type of the first message published to it.
      * If you try to publish a different type of message on the topic, it will be ignored.
      *
      * \warning Take care to always publish the same message type to the topic to avoid being spammed with errors.
      */
    template <class T>
    void publish(const std::string name, const T& msg);

  private:
    class impl;
    std::unique_ptr<impl> m_impl;
  };

#include <impl/dynamic_publisher.hpp>

}  // namespace mrs_lib

#endif  // DYNAMIC_PUBLISHER_H
