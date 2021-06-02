#ifndef MRS_TIMER_HPP
#define MRS_TIMER_HPP

// | ------------------------ ROSTimer ------------------------ |

/* ROSTimer constructors //{ */

// duration + oneshot + autostart
#include <chrono>
#include <mutex>
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent&),
                   ObjectType* const obj, const bool oneshot, const bool autostart) {

  this->timer_ = std::make_unique<ros::Timer>(nh.createTimer(duration, callback, obj, oneshot, autostart));
}

// rate + oneshot + autostart
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
                   const bool oneshot, const bool autostart) {

  this->timer_ = std::make_unique<ros::Timer>(nh.createTimer(rate, callback, obj, oneshot, autostart));
}

//}

// | ----------------------- ThreadTimer ---------------------- |

/* class ThreadTimer::Impl //{ */

class ThreadTimer::Impl {
public:
  Impl(const std::function<void(const ros::TimerEvent&)>& callback, const ros::Duration& delay_dur, const bool oneshot);
  ~Impl();

  void start();
  void stop();
  void setPeriod(const ros::Duration& duration, const bool reset = true);

  friend class ThreadTimer;

  // to keep rule of five since we have a custom destructor
  Impl(const Impl&) = delete;
  Impl(Impl&&) = delete;
  Impl& operator=(const Impl&) = delete;
  Impl& operator=(Impl&&) = delete;

private:
  std::thread thread_;
  std::function<void(const ros::TimerEvent&)> callback_;

  bool oneshot_;

  bool breakableSleep(const ros::Time& until);
  void threadFcn();

  std::mutex mutex_wakeup_;
  std::condition_variable wakeup_cond_;
  std::recursive_mutex mutex_state_;
  bool running_;
  ros::Duration delay_dur_;
  bool ending_;
  ros::Time next_expected_;
  ros::Time last_expected_;
  ros::Time last_real_;

};

//}

/* ThreadTimer constructors and destructors//{ */

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent&),
                         ObjectType* const obj, const bool oneshot, const bool autostart)
  : ThreadTimer(nh, rate.expectedCycleTime(), callback, obj, oneshot, autostart)
{
}

template <class ObjectType>
ThreadTimer::ThreadTimer([[maybe_unused]] const ros::NodeHandle& nh, const ros::Duration& duration,
                         void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj, bool oneshot, const bool autostart) 
{
  const auto cbk = std::bind(callback, obj, std::placeholders::_1);
  if (duration == ros::Duration(0))
    oneshot = true;
  this->impl_ = std::make_unique<Impl>(cbk, duration, oneshot);
  if (autostart)
    this->impl_->start();
}

//}

#endif  // MRS_TIMER_HPP
