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
#include <rclcpp/publisher.hpp>

#include "mrs_lib/internal/rate_throttle.hpp"
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

  public:
    Impl(const PublisherOptions& options, std::string_view topic_name)
        : node_interfaces_(options.node_interfaces),
          logger_(node_interfaces_),
          unresolved_topic_name_(topic_name),
          publisher_(rclcpp::create_publisher<MessageType>(node_interfaces_, std::string(topic_name), options.qos))
    {
      if (options.throttle_duration.has_value())
      {
        throttle_.emplace(internal::RateThrottle(options.throttle_duration.value()));
      }

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
     * published (see internal::RateThrottle).
     */
    bool handle_publish_rate()
    {
      if (!throttle_.has_value())
      {
        // Always publish if no throttle is set.
        return true;
      }

      auto guard = throttle_->acquire();
      return guard->accept(node_interfaces_.get_node_clock_interface()->get_clock()->now());
    }

    PublisherNodeInterfaces node_interfaces_;
    mrs_lib::Logger logger_;

    std::string unresolved_topic_name_;

    // empty if throttling is disabled, so unthrottled publishing skips the mutex and the clock read
    std::optional<OwningMutex<internal::RateThrottle>> throttle_;

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
