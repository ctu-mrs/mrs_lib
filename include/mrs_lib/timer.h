#ifndef MRS_TIMER_H
#define MRS_TIMER_H

#include <ros/ros.h>
#include <thread>
#include <mutex>

namespace mrs_lib
{

using callback_t = std::function<void(const ros::TimerEvent&)>;

// | ------------------------ ROSTimer ------------------------ |

/* class ROSTimer //{ */

class ROSTimer {
public:
  ROSTimer(void);
  ~ROSTimer(void);
  ROSTimer(const ROSTimer& other);

  ROSTimer& operator=(const ROSTimer& other);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot, const bool& autostart);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot, const bool& autostart);

  void stop(void);
  void start(void);
  void setPeriod(const ros::Duration& duration, const bool& reset = true);

  std::shared_ptr<ros::Timer> timer_;
};

//}

/* ROSTimer constructors //{ */

ROSTimer::ROSTimer(void) {
}

ROSTimer::~ROSTimer(void) {
}

ROSTimer::ROSTimer(const ROSTimer& other) {
  printf("muhaha constructor\n");
  this->timer_ = other.timer_;
}

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

/* ROSTimer:: operator= //{ */

ROSTimer& ROSTimer::operator=(const ROSTimer& other) {

  if (this == &other) {
    return *this;
  }

  this->timer_ = other.timer_;

  return *this;
}

/* //} */

/* stop() //{ */

void ROSTimer::stop(void) {
  timer_->stop();
}

//}

/* start() //{ */

void ROSTimer::start(void) {
  timer_->start();
}

//}

/* setPeriod() //{ */

void ROSTimer::setPeriod(const ros::Duration& duration, const bool& reset) {

  timer_->setPeriod(duration, reset);
}

//}

// | ----------------------- ThreadTimer ---------------------- |

/* class ThreadTimer //{ */

class ThreadTimer {

public:
  ThreadTimer();
  ~ThreadTimer();
  ThreadTimer(const ThreadTimer& other);

  ThreadTimer& operator=(const ThreadTimer& other);

  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj);

  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event),
              ObjectType* const obj);

  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot);

  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot);

  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot, const bool& autostart);

  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot, const bool& autostart);

  void stop(void);
  void start(void);
  void setPeriod(const ros::Duration& duration, const bool& reset = true);

private:
  class Impl {

  public:
    Impl();
    ~Impl();

    void start(void);
    void stop(void);
    void setPeriod(const ros::Duration& duration, const bool& reset = true);

    bool      oneshot_;
    ros::Rate rate_;

    callback_t callback_;

  private:
    bool      running_;
    ros::Time next_expected_;
    ros::Time last_expected_;
    ros::Time last_real_;

    std::thread thread_;

    bool       rate_changed_ = false;
    std::mutex mutex_rate_;

    void threadFcn(void) {

      if (oneshot_) {

        ros::TimerEvent timer_event;
        callback_(timer_event);

      } else {

        ros::Rate temp_rate = rate_;

        while (ros::ok() && running_) {

          ros::TimerEvent timer_event;
          timer_event.last_expected    = last_expected_;
          timer_event.last_real        = last_real_;
          timer_event.current_expected = next_expected_;
          timer_event.current_real     = ros::Time::now();

          callback_(timer_event);

          last_expected_ = next_expected_;
          last_real_     = ros::Time::now();

          next_expected_ = ros::Time::now() + rate_.expectedCycleTime();

          temp_rate.sleep();

          if (rate_changed_) {
            std::scoped_lock lock(mutex_rate_);
            temp_rate = rate_;
          }
        }
      }
    }
  };

  std::shared_ptr<Impl> impl_;

};  // namespace mrs_lib

//}

/* ThreadTimer constructors and destructors//{ */

ThreadTimer::ThreadTimer() {
}

ThreadTimer::~ThreadTimer() {
}

ThreadTimer::ThreadTimer(const ThreadTimer& other) {

  impl_ = other.impl_;
}

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

/* ThreadTimer operator= //{ */

ThreadTimer& ThreadTimer::operator=(const ThreadTimer& other) {

  if (this == &other) {
    return *this;
  }

  this->impl_ = other.impl_;

  return *this;
}

//}

/* TheadTimer::start() //{ */

void ThreadTimer::start(void) {

  if (impl_) {
    impl_->start();
  }
}

//}

/* ThreadTimer::stop() //{ */

void ThreadTimer::stop(void) {

  if (impl_) {
    impl_->stop();
  }
}

//}

/* ThreadTimer::setPeriod() //{ */

void ThreadTimer::setPeriod(const ros::Duration& duration, [[maybe_unused]] const bool& reset) {

  if (impl_) {
    impl_->setPeriod(duration, reset);
  }
}

//}

/* ThreadTimer::Impl //{ */

ThreadTimer::Impl::~Impl() {
}

ThreadTimer::Impl::Impl() : rate_(ros::Rate(1.0)) {

  this->running_ = false;
  this->oneshot_ = false;

  this->last_real_     = ros::Time(0);
  this->last_expected_ = ros::Time(0);
  this->next_expected_ = ros::Time(0);
}

void ThreadTimer::Impl::start(void) {

  this->stop();

  if (!running_) {
    running_ = true;
    thread_  = std::thread(&Impl::threadFcn, this);
  }
}

void ThreadTimer::Impl::stop(void) {

  running_ = false;

  if (thread_.joinable()) {
    thread_.join();
  }
}

void ThreadTimer::Impl::setPeriod(const ros::Duration& duration, [[maybe_unused]] const bool& reset) {

  std::scoped_lock lock(mutex_rate_);

  if (duration.toSec() > 0) {
    this->rate_ = 1.0 / duration.toSec();
  } else {
    this->oneshot_ = true;
    this->rate_    = ros::Rate(1.0);
  }

  rate_changed_ = true;
}

//}

}  // namespace mrs_lib

#endif  // MRS_TIMER_H
