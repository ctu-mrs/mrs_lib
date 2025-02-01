// clang: TomasFormat
#ifndef PUBLISHER_HANDLER_HPP
#define PUBLISHER_HANDLER_HPP

namespace mrs_lib
{

// --------------------------------------------------------------
// |                    PublisherHandler_impl                   |
// --------------------------------------------------------------

/* PublisherHandler_impl(void) //{ */

template <class TopicType>
PublisherHandler_impl<TopicType>::PublisherHandler_impl(void) : publisher_initialized_(false) {
}

//}

/* PublisherHandler_impl(std::shared_ptr<rclcpp::Node> node, const std::string& address, const unsigned int& buffer_size, const bool& latch, const double& rate)
 * //{ */

template <class TopicType>
PublisherHandler_impl<TopicType>::PublisherHandler_impl(std::shared_ptr<rclcpp::Node> node, const std::string& address, const unsigned int& buffer_size,
                                                        const bool& latch, const double& rate) {

  {
    std::scoped_lock lock(mutex_publisher_);

    address_   = address;
    publisher_ = node->create_publisher<TopicType>(address, buffer_size);
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

    publisher_->publish(msg);
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

    publisher_->publish(msg);
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

    publisher_->publish(msg);
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

/* PublisherHandler(std::shared_ptr<rclcpp::Node> node, const std::string& address, const unsigned int& buffer_size, const bool& latch,
                                              const double& rate) //{ */

template <class TopicType>
PublisherHandler<TopicType>::PublisherHandler(std::shared_ptr<rclcpp::Node> node, const std::string& address, const unsigned int& buffer_size,
                                              const bool& latch, const double& rate) {

  impl_ = std::make_shared<PublisherHandler_impl<TopicType>>(node, address, buffer_size, latch, rate);
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
