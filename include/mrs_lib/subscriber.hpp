#ifndef MRS_LIB_SUBSCRIBER_HPP_
#define MRS_LIB_SUBSCRIBER_HPP_


#include <chrono>
#include <memory>
#include <optional>
#include <string_view>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#include "mrs_lib/timeout_manager.hpp"
#include "mrs_lib/utility/callback.hpp"
#include "mrs_lib/utility/pimpl.hpp"


namespace mrs_lib
{

  /**
   * @brief ROS node interfaces required by the Subscriber.
   */
  using SubscriberNodeInterfaces =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeTimersInterface,
                                              rclcpp::node_interfaces::NodeClockInterface, rclcpp::node_interfaces::NodeLoggingInterface,
                                              rclcpp::node_interfaces::NodeParametersInterface, rclcpp::node_interfaces::NodeTopicsInterface>;

  /**
   * @brief Options for configuring the Subscriber.
   */
  struct SubscriberOptions
  {
    /** @brief ROS node interfaces used by the subscriber. */
    SubscriberNodeInterfaces node_interfaces;

    /** @brief QOS settings for the subscriber. */
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
    /** @brief ROS callback group used by the subscriber callbacks. */
    std::shared_ptr<rclcpp::CallbackGroup> callback_group = nullptr;

    /** @brief Time after which timeout callback is called if no message is received. */
    std::optional<std::chrono::nanoseconds> no_message_timeout = std::nullopt;
    /**
     * @brief Timeout manager used for the no message timeout.
     *
     * If nullptr and timeout is set, the Subscriber will create its own timeout manager.
     */
    std::shared_ptr<mrs_lib::TimeoutManager2> timeout_manager = nullptr;
    /**
     * @brief Timeout manager used for the no message timeout.
     *
     * If not set, a default will be used.
     */
    std::function<void(std::string_view topic_name, rclcpp::Time last_msg)> timeout_callback = {};
  };

  /**
   * @brief Tag for explicitly marking Subscriber that has no callback.
   */
  struct PolledSubscriberTag
  {
  };

  /**
   * @brief Wrapper for rclcpp::Subscription with callback and polling interface.
   *
   * @tparam MessageType Type of the message to subscribe to.
   */
  template <typename MessageType>
  class Subscriber
  {
  private:
    class Impl;

  public:
    /** @brief Type of the accepted callback. */
    using MessageCallback = std::function<void(std::shared_ptr<const MessageType>)>;

    /** @brief Type of the accepted coroutine callback. */
    using MessageCoroCallback = CoroCallback<void(std::shared_ptr<const MessageType>)>;

    /**
     * @brief Construct a Subscriber with the specified callback.
     *
     * @param options Options to configure the subscriber.
     * @param topic_name Name of the topic to subscribe to.
     * @param callback Callback to call when a message is received.
     */
    explicit Subscriber(const SubscriberOptions& options, std::string_view topic_name, MessageCallback callback);

    /**
     * @brief Construct a Subscriber with the specified coroutine callback.
     *
     * @param options Options to configure the subscriber.
     * @param topic_name Name of the topic to subscribe to.
     * @param callback Coroutine callback to call when a message is received.
     */
    explicit Subscriber(const SubscriberOptions& options, std::string_view topic_name, MessageCoroCallback callback);

    /**
     * @brief Construct a Subscriber without any callback.
     *
     * @param options Options to configure the subscriber.
     * @param topic_name Name of the topic to subscribe to.
     * @param tag Tag to explicitly specify that this subscriber has no callback.
     */
    explicit Subscriber(const SubscriberOptions& options, std::string_view topic_name, PolledSubscriberTag tag);

    /**
     * @brief Get the latest received message, if there is one.
     *
     * @return The latest message, if there is any, nullopt otherwise.
     *
     * Calling this function marks the message as used.
     */
    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> get_message();

    /**
     * @brief Get the latest new message, if there is one.
     *
     * @return The latest new message, if there is any, nullopt otherwise.
     *
     * A new message is a message that was not yet used.
     * A message is always marked as used if there is a callback associated with
     * the subscriber.
     * Additionally, it is marked as used by obtaining it using get_*message.
     *
     * Calling this function marks the message as used.
     */
    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> get_new_message();

    /**
     * @brief Get the latest received message, if there is one.
     *
     * @return The latest message, if there is any, nullopt otherwise.
     *
     * Calling this function does NOT mark the message as used.
     */
    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> peek_message() const;

    /**
     * @brief Get the latest new message, if there is one.
     *
     * @return The latest new message, if there is any, nullopt otherwise.
     *
     * A new message is a message that was not yet used.
     * A message is always marked as used if there is a callback associated with
     * the subscriber.
     * Additionally, it is marked as used by obtaining it using get_*message.
     *
     * Calling this function does NOT mark the message as used.
     */
    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> peek_new_message() const;


    /**
     * @brief Returns time of the last received message on the topic, handled by this Subscriber.
     *
     * @return time when the last message was received.
     */
    [[nodiscard]] rclcpp::Time get_last_message_time() const;

    /**
     * @brief Returns the resolved (full) name of the topic handled by this Subscriber.
     *
     * @return The name of the handled topic.
     */
    [[nodiscard]] std::string get_topic_name() const;

    /**
     * @brief Returns the unresolved name of the topic handled by this Subscriber.
     *
     * @return The name of the handled topic.
     */
    [[nodiscard]] std::string get_unresolved_topic_name() const;

    /**
     * @brief Returns number of publishers registered at the topic.
     *
     * @return number of publishers.
     */
    [[nodiscard]] uint32_t get_publisher_count() const;

  private:
    Pimpl<Impl> impl_;
  };

} // namespace mrs_lib

#ifndef MRS_LIB_SUBSCRIBER_IMPL_HPP_
#include "mrs_lib/subscriber.impl.hpp"
#endif

#endif // MRS_LIB_SUBSCRIBER_HPP_
