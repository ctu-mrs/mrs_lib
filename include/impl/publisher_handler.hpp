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

/* PublisherHandler_impl(ros::NodeHandle& nh, const std::string& address, const unsigned int &buffer_size, const bool &latch) //{ */

template <class TopicType>
PublisherHandler_impl<TopicType>::PublisherHandler_impl(ros::NodeHandle& nh, const std::string& address, const unsigned int& buffer_size, const bool& latch) {

  {
    std::scoped_lock lock(mutex_publisher_);

    publisher_ = nh.advertise<TopicType>(address, buffer_size, latch);
  }

  _address_ = address;

  publisher_initialized_ = true;
}

//}

/* publish(TopicType& msg) //{ */

template <class TopicType>
void PublisherHandler_impl<TopicType>::publish(const TopicType& msg) {

  if (!publisher_initialized_) {
    return;
  }

  try {
    publisher_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_.getTopic().c_str());
  }
}

//}

/* publish(const boost::shared_ptr<TopicType>& msg) //{ */

template <class TopicType>
void PublisherHandler_impl<TopicType>::publish(const boost::shared_ptr<TopicType>& msg) {

  if (!publisher_initialized_) {
    return;
  }

  try {
    publisher_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_.getTopic().c_str());
  }
}

//}

/* publish(const boost::shared_ptr<TopicType const>& msg) //{ */

template <class TopicType>
void PublisherHandler_impl<TopicType>::publish(const boost::shared_ptr<TopicType const>& msg) {

  if (!publisher_initialized_) {
    return;
  }

  try {
    publisher_.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_.getTopic().c_str());
  }
}

//}

/* getNumSubscribers(void) //{ */

template <class TopicType>
unsigned int PublisherHandler_impl<TopicType>::getNumSubscribers(void) {

  return publisher_.getNumSubscribers();
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

/* PublisherHandler(ros::NodeHandle& nh, const std::string& address, const unsigned int &buffer_size, const bool &latch) //{ */

template <class TopicType>
PublisherHandler<TopicType>::PublisherHandler(ros::NodeHandle& nh, const std::string& address, const unsigned int& buffer_size, const bool& latch) {

  impl_ = std::make_shared<PublisherHandler_impl<TopicType>>(nh, address, buffer_size, latch);
}

//}

/* initialize(ros::NodeHandle& nh, const std::string& address) //{ */

template <class TopicType>
void PublisherHandler<TopicType>::initialize(ros::NodeHandle& nh, const std::string& address) {

  impl_ = std::make_shared<PublisherHandler_impl<TopicType>>(nh, address);
}

//}

/* publish(const TopicType& msg) //{ */

template <class TopicType>
void PublisherHandler<TopicType>::publish(const TopicType& msg) {

  impl_->publish(msg);
}

//}

/* publish(const boost::shared_ptr<TopicType>& msg) //{ */

template <class TopicType>
void PublisherHandler<TopicType>::publish(const boost::shared_ptr<TopicType>& msg) {

  impl_->publish(msg);
}

//}

/* publish(const boost::shared_ptr<TopicType const>& msg) //{ */

template <class TopicType>
void PublisherHandler<TopicType>::publish(const boost::shared_ptr<TopicType const>& msg) {

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
