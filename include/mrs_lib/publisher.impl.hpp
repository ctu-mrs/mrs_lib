#ifndef MRS_LIB_PUBLISHER_IMPL_HPP_
#define MRS_LIB_PUBLISHER_IMPL_HPP_

#include "mrs_lib/publisher.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/publisher.hpp>

#include "mrs_lib/logger.hpp"
#include "mrs_lib/utility/owning_mutex.hpp"
#include "mrs_lib/utility/pimpl.hpp"


namespace mrs_lib
{


  /** @private */
  template <class MessageType>
  class Publisher<MessageType>::Impl
  {
  private:
    using RosPublisherPimpl = Pimpl<rclcpp::Publisher<MessageType>, std::shared_ptr<rclcpp::Publisher<MessageType>>>;

    struct ThrottleState
    {
      rclcpp::Time last_time_published;
    };

  public:
    Impl(const PublisherOptions& options, std::string_view topic_name)
        : node_interfaces_(options.node_interfaces),
          logger_(node_interfaces_),
          unresolved_topic_name_(topic_name),
          throttle_duration_(options.throttle_duration),
          throttle_state_(ThrottleState{
              .last_time_published = node_interfaces_.get_node_clock_interface()->get_clock()->now(),
              // .publisher = RosPublisherPimpl(rclcpp::create_publisher<MessageType>(node_interfaces_, topic_name, options.qos)),
          }),
          publisher_(rclcpp::create_publisher<MessageType>(node_interfaces_, std::string(topic_name), options.qos))
    {
      logger_.info("Created publisher on topic '{}' -> '{}'", unresolved_topic_name_, publisher_->get_topic_name());
    }

    ~Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    template <typename Message>
    void publish(Message&& msg)
    {
      bool should_publish = handle_publish_rate();

      if (should_publish)
      {
        publisher_->publish(std::forward<Message>(msg));
      }
    }

    [[nodiscard]] std::string get_topic_name() const
    {
      return publisher_->get_topic_name();
    }

    [[nodiscard]] std::string get_unresolved_topic_name() const
    {
      return unresolved_topic_name_;
    }

    [[nodiscard]] size_t get_subscriber_count() const
    {
      return publisher_->get_subscription_count();
    }

  private:
    /**
     * @brief Handle publish throttle.
     *
     * @param state Unlocked publisher state.
     *
     * @return `true` if the message should be published, `false` otherwise.
     *
     * If publisher throttling is disabled, does nothing and returns true.
     *
     * If publisher throttling is enabled, checks whether the message should be
     * published.
     * If it should, it also updates the last published time.
     */
    bool handle_publish_rate()
    {
      auto guard = throttle_state_.acquire();
      rclcpp::Time now = node_interfaces_.get_node_clock_interface()->get_clock()->now();

      if (!throttle_duration_.has_value())
      {
        // Always publish if no throttle is set.
        return true;
      }

      rclcpp::Duration passed = now - guard->last_time_published;

      if (passed < throttle_duration_.value())
      {
        return false;
      }

      // Throttle is enabled and message will be published.
      guard->last_time_published = now;
      return true;
    }

    PublisherNodeInterfaces node_interfaces_;
    mrs_lib::Logger logger_;

    std::string unresolved_topic_name_;

    std::optional<std::chrono::nanoseconds> throttle_duration_;

    OwningMutex<ThrottleState> throttle_state_;

    RosPublisherPimpl publisher_;
  };


  template <class MessageType>
  Publisher<MessageType>::Publisher(const PublisherOptions& options, std::string_view topic_name) : impl_(std::in_place_type_t<Impl>{}, options, topic_name)
  {
  }

  template <class MessageType>
  void Publisher<MessageType>::publish(const MessageType& msg)
  {
    impl_->publish(msg);
  }

  template <class MessageType>
  void Publisher<MessageType>::publish(std::unique_ptr<MessageType> msg)
  {
    impl_->publish(std::move(msg));
  }

  template <class MessageType>
  [[nodiscard]] std::string Publisher<MessageType>::get_topic_name() const
  {
    return impl_->get_topic_name();
  }

  template <class MessageType>
  [[nodiscard]] std::string Publisher<MessageType>::get_unresolved_topic_name() const
  {
    return impl_->get_unresolved_topic_name();
  }

  template <class MessageType>
  size_t Publisher<MessageType>::get_subscriber_count() const
  {
    return impl_->get_subscriber_count();
  }

} // namespace mrs_lib

#endif // MRS_LIB_PUBLISHER_IMPL_HPP_
