#ifndef MRS_LIB_SUBSCRIBER_IMPL_HPP_
#define MRS_LIB_SUBSCRIBER_IMPL_HPP_

#include "mrs_lib/subscriber.hpp"

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include <rclcpp/create_subscription.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/time.hpp>

#include "mrs_lib/internal/coroutine_callback_helpers.hpp"
#include "mrs_lib/logger.hpp"
#include "mrs_lib/timeout_manager.hpp"
#include "mrs_lib/utility/owning_mutex.hpp"
#include "mrs_lib/utility/pimpl.hpp"

namespace mrs_lib
{
  /** @private */
  template <typename MessageType>
  class Subscriber<MessageType>::Impl
  {
    struct TimeoutData
    {
      std::shared_ptr<TimeoutManager2> timeout_manager{};
      std::shared_ptr<TimeoutManagerHandle> timeout_handle{};
    };

    struct LatestMessageData
    {
      rclcpp::Time time{};
      std::shared_ptr<const MessageType> message{};
      bool is_new = false;
    };

  public:
    explicit Impl(const SubscriberOptions& options, std::string_view topic_name, std::optional<MessageCallback> callback)
        : node_interfaces_(options.node_interfaces),
          logger_(node_interfaces_),
          unresolved_name_(topic_name),
          timeout_data_(init_timeout_data(options, topic_name)),
          user_callback_(check_nonempty_callback_function(std::move(callback))),
          subscription_(rclcpp::create_subscription<MessageType>(
              node_interfaces_, std::string(topic_name), options.qos,
              std::function<void(std::shared_ptr<const MessageType>)>(std::bind_front(&Impl::message_callback, this)), std::invoke([&]() {
                rclcpp::SubscriptionOptions opts{};
                opts.callback_group = options.callback_group;
                return opts;
              })))
    {
      logger_.info("Subscribed to topic '{}' -> '{}'", this->get_unresolved_topic_name(), this->get_topic_name());
    }

    explicit Impl(const SubscriberOptions& options, std::string_view topic_name, MessageCoroCallback callback)
        : Impl(options, topic_name, internal::get_detached_coro_callback_launcher(callback, options.callback_group))
    {
    }

    ~Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> get_message()
    {
      auto guard = latest_message_data_.acquire();
      guard->is_new = false;

      if (guard->message != nullptr)
      {
        return guard->message;
      } else
      {
        return std::nullopt;
      }
    }

    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> get_new_message()
    {
      auto guard = latest_message_data_.acquire();

      if (guard->is_new && guard->message != nullptr)
      {
        guard->is_new = false;
        return guard->message;
      } else
      {
        return std::nullopt;
      }
    }

    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> peek_message() const
    {
      auto guard = latest_message_data_.acquire();
      if (guard->message != nullptr)
      {
        return guard->message;
      } else
      {
        return std::nullopt;
      }
    }

    [[nodiscard]] std::optional<std::shared_ptr<const MessageType>> peek_new_message() const
    {
      auto guard = latest_message_data_.acquire();

      if (guard->is_new && guard->message != nullptr)
      {
        return guard->message;
      } else
      {
        return std::nullopt;
      }
    }

    [[nodiscard]] rclcpp::Time get_last_message_time() const
    {
      auto guard = latest_message_data_.acquire();
      return guard->time;
    };

    [[nodiscard]] std::string get_topic_name() const
    {
      return subscription_->get_topic_name();
    }

    [[nodiscard]] std::string get_unresolved_topic_name() const
    {
      return unresolved_name_;
    }

    [[nodiscard]] size_t get_publisher_count() const
    {
      return subscription_->get_publisher_count();
    };

  private:
    static std::optional<MessageCallback> check_nonempty_callback_function(std::optional<MessageCallback> callback)
    {
      if (callback.has_value() && !callback.value())
      {
        throw std::logic_error("Subscriber callback must not be empty.");
      } else
      {
        return std::move(callback);
      }
    }

    TimeoutData init_timeout_data(const SubscriberOptions& options, std::string_view topic_name)
    {
      if (!options.no_message_timeout.has_value())
      {
        return {};
      }

      auto no_message_timeout = options.no_message_timeout.value();

      auto timeout_callback = std::invoke([&]() -> TimeoutManager2::Callback {
        if (options.timeout_callback)
        {
          return [callback = options.timeout_callback, name = std::string(topic_name)](rclcpp::Time time) { callback(name, time); };
        } else
        {
          return std::bind_front(&Impl::default_timeout_callback, this, std::string(topic_name));
        }
      });


      std::shared_ptr<mrs_lib::TimeoutManager2> timeout_manager = options.timeout_manager;
      // initialize a new TimeoutManager if not provided by the user
      if (timeout_manager == nullptr)
      {
        timeout_manager = std::make_shared<mrs_lib::TimeoutManager2>(node_interfaces_, no_message_timeout / 2);
      }

      // register the timeout callback with the TimeoutManager
      auto timeout_handle = timeout_manager->register_new({
          .timeout = no_message_timeout,
          .callback = timeout_callback,
      });

      return {
          .timeout_manager = timeout_manager,
          .timeout_handle = timeout_handle,
      };
    }

    void default_timeout_callback(std::string_view, rclcpp::Time last_msg)
    {
      const rclcpp::Duration since_msg = (node_interfaces_.get_node_clock_interface()->get_clock()->now() - last_msg);
      const auto n_pubs = subscription_->get_publisher_count();
      logger_.warning("Did not receive any message from topic '{}' for {}s ({} publishers on this topic)", get_topic_name(), since_msg.seconds(),
                      std::to_string(n_pubs));
    }

    void process_new_message(const std::shared_ptr<const MessageType>& msg)
    {
      auto latest_message_data_guard = latest_message_data_.acquire();
      latest_message_data_guard->time = node_interfaces_.get_node_clock_interface()->get_clock()->now();
      latest_message_data_guard->message = msg;
      // If the message callback is registered, the new data will immediately be processed,
      // so reset the flag. Otherwise, set the flag.
      latest_message_data_guard->is_new = !user_callback_.has_value();
    }

    void message_callback(const std::shared_ptr<const MessageType>& msg)
    {
      {
        auto timeout_data_guard = timeout_data_.acquire();

        if (timeout_data_guard->timeout_manager && timeout_data_guard->timeout_handle)
        {
          timeout_data_guard->timeout_handle->reset_timeout();
        }
      }

      process_new_message(msg);

      // execute the callback after unlocking the mutex to enable multi-threaded callback execution
      if (user_callback_.has_value())
      {
        (*user_callback_)(msg);
      }
    }

  private:
    SubscriberNodeInterfaces node_interfaces_;
    Logger logger_;

    std::string unresolved_name_;

    OwningMutex<TimeoutData> timeout_data_;
    OwningMutex<LatestMessageData> latest_message_data_;

    std::optional<MessageCallback> user_callback_;

    Pimpl<rclcpp::Subscription<MessageType>, std::shared_ptr<rclcpp::Subscription<MessageType>>> subscription_;
  };

  template <typename MessageType>
  inline Subscriber<MessageType>::Subscriber(const SubscriberOptions& options, std::string_view topic_name, MessageCallback callback)
      : impl_(std::in_place_type_t<Impl>{}, options, topic_name, std::move(callback))
  {
  }

  template <typename MessageType>
  inline Subscriber<MessageType>::Subscriber(const SubscriberOptions& options, std::string_view topic_name, MessageCoroCallback callback)
      : impl_(std::in_place_type_t<Impl>{}, options, topic_name, std::move(callback))
  {
  }

  template <typename MessageType>
  inline Subscriber<MessageType>::Subscriber(const SubscriberOptions& options, std::string_view topic_name, PolledSubscriberTag)
      : impl_(std::in_place_type_t<Impl>{}, options, topic_name, std::nullopt)
  {
  }

  template <typename MessageType>
  std::optional<std::shared_ptr<const MessageType>> Subscriber<MessageType>::get_message()
  {
    return impl_->get_message();
  };

  template <typename MessageType>
  std::optional<std::shared_ptr<const MessageType>> Subscriber<MessageType>::get_new_message()
  {
    return impl_->get_new_message();
  };

  template <typename MessageType>
  std::optional<std::shared_ptr<const MessageType>> Subscriber<MessageType>::peek_message() const
  {
    return impl_->peek_message();
  };

  template <typename MessageType>
  std::optional<std::shared_ptr<const MessageType>> Subscriber<MessageType>::peek_new_message() const
  {
    return impl_->peek_message();
  };

  template <typename MessageType>
  rclcpp::Time Subscriber<MessageType>::get_last_message_time() const
  {
    return impl_->get_last_message_time();
  };

  template <typename MessageType>
  std::string Subscriber<MessageType>::get_topic_name() const
  {
    return impl_->get_topic_name();
  };

  template <typename MessageType>
  std::string Subscriber<MessageType>::get_unresolved_topic_name() const
  {
    return impl_->get_unresolved_topic_name();
  };

  template <typename MessageType>
  uint32_t Subscriber<MessageType>::get_publisher_count() const
  {
    return impl_->get_publisher_count();
  };

} // namespace mrs_lib

#endif // MRS_LIB_SUBSCRIBER_IMPL_HPP_
