// clang: TomasFormat
#ifndef PUBLISHER_HANDLER_HPP
#define PUBLISHER_HANDLER_HPP

#include <mrs_lib/publisher_handler.h>

namespace mrs_lib
{

// --------------------------------------------------------------
// |                    PublisherHandler_impl                   |
// --------------------------------------------------------------

/* PublisherHandler_impl(void) //{ */

template <class TopicType>
PublisherHandler_impl<TopicType>::PublisherHandler_impl(void) : publisher_initialized_(false) {

  last_time_published_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

//}

/* PublisherHandler_impl(const PublisherHandlerOptions& options, const std::string& address) //{ */

template <class TopicType>
PublisherHandler_impl<TopicType>::PublisherHandler_impl(const PublisherHandlerOptions& options, const std::string& address) {

  node_ = options.node;

  last_time_published_ = node_->get_clock()->now();

  {
    std::scoped_lock lock(mutex_publisher_);

    address_   = address;
    publisher_ = node_->create_publisher<TopicType>(address, options.qos);
  }

  if (options.throttle_rate > 1e-3) {
    this->throttle_min_dt_ = 1.0 / options.throttle_rate;
    this->throttle_        = true;
  }

  publisher_initialized_ = true;
}

//}

/* publish(TopicType& msg) //{ */

template <class TopicType>
void PublisherHandler_impl<TopicType>::publish(const TopicType& msg) {

  if (!publisher_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_publisher_);

    rclcpp::Time now = node_->get_clock()->now();

    double passed = (now - last_time_published_).seconds();

    if (passed > throttle_min_dt_) {
      publisher_->publish(msg);
      last_time_published_ = now;
    }
  }
}

//}

/* publish(const std::shared_ptr<TopicType>& msg) //{ */

template <class TopicType>
void PublisherHandler_impl<TopicType>::publish(const std::shared_ptr<TopicType>& msg) {

  if (!publisher_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_publisher_);

    rclcpp::Time now = node_->get_clock()->now();

    double passed = (now - last_time_published_).seconds();

    if (passed > throttle_min_dt_) {
      publisher_->publish(msg);
      last_time_published_ = now;
    }
  }
}

//}

/* publish(const std::shared_ptr<TopicType const>& msg) //{ */

template <class TopicType>
void PublisherHandler_impl<TopicType>::publish(const std::shared_ptr<TopicType const>& msg) {

  if (!publisher_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_publisher_);

    rclcpp::Time now = node_->get_clock()->now();

    double passed = (now - last_time_published_).seconds();

    if (passed > throttle_min_dt_) {
      publisher_->publish(msg);
      last_time_published_ = now;
    }
  }
}

//}

/* getNumSubscribers(void) //{ */

template <class TopicType>
unsigned int PublisherHandler_impl<TopicType>::getNumSubscribers(void) {

  {
    std::scoped_lock lock(mutex_publisher_);

    return publisher_.getNumSubscribers();
  }
}

//}

// --------------------------------------------------------------
// |                      PublisherHandler                      |
// --------------------------------------------------------------

/* operator= //{ */

template <class TopicType>
PublisherHandler<TopicType>& PublisherHandler<TopicType>::operator=(const PublisherHandler<TopicType>& other) {

  if (this == &other) {
    return *this;
  }

  if (other.impl_) {
    this->impl_ = other.impl_;
  }

  return *this;
}

//}

/* copy constructor //{ */

template <class TopicType>
PublisherHandler<TopicType>::PublisherHandler(const PublisherHandler<TopicType>& other) {

  if (other.impl_) {
    this->impl_ = other.impl_;
  }
}

//}

/* PublisherHandler(std::shared_ptr<rclcpp::Node> node, const std::string& address) //{ */

template <class TopicType>
PublisherHandler<TopicType>::PublisherHandler(std::shared_ptr<rclcpp::Node> node, const std::string& address) {

  PublisherHandlerOptions opts;

  opts.node = node;

  impl_ = std::make_shared<PublisherHandler_impl<TopicType>>(opts, address);
}

//}

/* PublisherHandler(const rclcpp::PublisherOptions& options, const std::string& address) //{ */

template <class TopicType>
PublisherHandler<TopicType>::PublisherHandler(const PublisherHandlerOptions& options, const std::string& address) {

  impl_ = std::make_shared<PublisherHandler_impl<TopicType>>(options, address);
}

//}

/* publish(const TopicType& msg) //{ */

template <class TopicType>
void PublisherHandler<TopicType>::publish(const TopicType& msg) {

  impl_->publish(msg);
}

//}

/* publish(const std::shared_ptr<TopicType>& msg) //{ */

template <class TopicType>
void PublisherHandler<TopicType>::publish(const std::shared_ptr<TopicType>& msg) {

  impl_->publish(msg);
}

//}

/* publish(const std::shared_ptr<TopicType const>& msg) //{ */

template <class TopicType>
void PublisherHandler<TopicType>::publish(const std::shared_ptr<TopicType const>& msg) {

  impl_->publish(msg);
}

//}

/* getNumSubscribers(void) //{ */

template <class TopicType>
unsigned int PublisherHandler<TopicType>::getNumSubscribers(void) {

  return impl_->getNumSubscribers();
}

//}

}  // namespace mrs_lib

#endif  // PUBLISHER_HANDLER_HPP
