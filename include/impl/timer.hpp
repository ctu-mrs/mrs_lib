#ifndef MRS_TIMER_HPP
#define MRS_TIMER_HPP

// | ------------------------ ROSTimer ------------------------ |

/* ROSTimer constructors //{ */

// rate
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj) {
  *this = ROSTimer(nh, rate, callback, obj, false, true);
}

// duration
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                   ObjectType* const obj) {
  *this = ROSTimer(nh, duration, callback, obj, false, true);
}

// rate + oneshot
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
                   const bool& oneshot) {
  *this = ROSTimer(nh, rate, callback, obj, oneshot, true);
}

// duration + oneshot
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                   ObjectType* const obj, const bool& oneshot) {
  *this = ROSTimer(nh, duration, callback, obj, oneshot, true);
}

// duration + oneshot + autostart
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                   ObjectType* const obj, const bool& oneshot, const bool& autostart) {

  this->timer_ = std::make_shared<ros::Timer>(nh.createTimer(duration, callback, obj, oneshot, autostart));
}

// rate + oneshot + autostart
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
                   const bool& oneshot, const bool& autostart) {

  this->timer_ = std::make_shared<ros::Timer>(nh.createTimer(rate, callback, obj, oneshot, autostart));
}

//}

// | ----------------------- ThreadTimer ---------------------- |

/* ThreadTimer constructors and destructors//{ */

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                         ObjectType* const obj) {

  *this = ThreadTimer(nh, rate, callback, obj, false, true);
}

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Duration&                    duration,
                         void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj) {

  *this = ThreadTimer(nh, duration, callback, obj, false, true);
}

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                         ObjectType* const obj, const bool& oneshot) {

  *this = ThreadTimer(nh, rate, callback, obj, oneshot, true);
}

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Duration&                    duration,
                         void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj, const bool& oneshot) {

  *this = ThreadTimer(nh, duration, callback, obj, oneshot, true);
}

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                         ObjectType* const obj, const bool& oneshot, const bool& autostart) {

  this->impl_            = std::make_shared<Impl>();
  this->impl_->callback_ = std::bind(callback, obj, std::placeholders::_1);
  this->impl_->oneshot_  = oneshot;
  this->impl_->rate_     = rate;
  this->impl_->duration_ = rate.expectedCycleTime();

  if (autostart) {
    this->impl_->start();
  }
}

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Duration&                    duration,
                         void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj, const bool& oneshot, const bool& autostart) {

  this->impl_            = std::make_shared<Impl>();
  this->impl_->callback_ = std::bind(callback, obj, std::placeholders::_1);
  this->impl_->oneshot_  = oneshot;
  this->impl_->duration_ = duration;

  if (duration.toSec() > 0) {
    this->impl_->rate_ = 1.0 / duration.toSec();
  } else {
    this->impl_->oneshot_ = true;
  }

  if (autostart) {
    this->impl_->start();
  }
}

//}

#endif  // MRS_TIMER_HPP
