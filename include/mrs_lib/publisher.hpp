#ifndef MRS_LIB_PUBLISHER_HPP_
#define MRS_LIB_PUBLISHER_HPP_


#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include "mrs_lib/utility/pimpl.hpp"


namespace mrs_lib
{

  /**
   * @brief ROS node interfaces required by the Publisher.
   */
  using PublisherNodeInterfaces =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeTopicsInterface, rclcpp::node_interfaces::NodeClockInterface,
                                              rclcpp::node_interfaces::NodeLoggingInterface>;


  struct PublisherOptions
  {
    /** @brief ROS node interfaces used by the publisher. */
    PublisherNodeInterfaces node_interfaces;
    /** @brief QOS settings for the publisher. */
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

    /** @brief If set, the publisher will drop messages published within this time of a previous message. */
    std::optional<std::chrono::nanoseconds> throttle_duration = std::nullopt;
  };

  /**
   * @brief Wrapper for rclcpp::Publisher.
   *
   * @tparam MessageType Type of the message to publish.
   */
  template <class MessageType>
  class Publisher
  {
  private:
    class Impl;

  public:
    /**
     * @brief construct a publisher using the specified options.
     *
     * @param options Configuration of the publisher.
     * @param topic_name Name of the topic to publish to.
     */
    Publisher(const PublisherOptions& options, std::string_view topic_name);

    /**
     * @brief Publish a message.
     *
     * @param msg Message to publish.
     */
    void publish(const MessageType& msg);

    /**
     * @brief Publish a message.
     *
     * @param msg Message to publish.
     */
    void publish(std::unique_ptr<MessageType> msg);

    /**
     * @brief Returns the resolved (full) name of the topic handled by this publisher.
     *
     * @return The name of the handled topic.
     */
    [[nodiscard]] std::string get_topic_name() const;

    /**
     * @brief Returns the unresolved name of the topic handled by this publisher.
     *
     * @return The name of the handled topic.
     */
    [[nodiscard]] std::string get_unresolved_topic_name() const;

    /**
     * @brief Get number of subscribers on the published topic.
     *
     * @return The number of subscribers.
     */
    [[nodiscard]] size_t get_subscriber_count() const;

  private:
    Pimpl<Impl> impl_;
  };

} // namespace mrs_lib

#ifndef MRS_LIB_PUBLISHER_IMPL_HPP_
#include "mrs_lib/publisher.impl.hpp"
#endif

#endif // MRS_LIB_PUBLISHER_HPP_
