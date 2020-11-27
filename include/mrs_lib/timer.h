#ifndef MRS_TIMER_H
#define MRS_TIMER_H

#include <ros/ros.h>

namespace mrs_lib
{

// | ------------------------ ROSTimer ------------------------ |

/* class ROSTimer //{ */

class ROSTimer {
public:
  ROSTimer(void);
  ~ROSTimer(void);
  ROSTimer(const ROSTimer& other);

  ROSTimer& operator=(const ROSTimer& other);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot, const bool& autostart);

  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot, const bool& autostart);

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
  bool oneshot   = false;
  bool autostart = true;
  *this          = ROSTimer(nh, rate, callback, obj, oneshot, autostart);
}

// duration
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                   ObjectType* const obj) {
  bool oneshot   = false;
  bool autostart = true;
  *this          = ROSTimer(nh, duration, callback, obj, oneshot, autostart);
}

// rate + oneshot
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
                   const bool& oneshot) {
  bool autostart = true;
  *this          = ROSTimer(nh, rate, callback, obj, oneshot, autostart);
}

// duration + oneshot
template <class ObjectType>
ROSTimer::ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event),
                   ObjectType* const obj, const bool& oneshot) {
  bool autostart = true;
  *this          = ROSTimer(nh, duration, callback, obj, oneshot, autostart);
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

}  // namespace mrs_lib

#endif  // MRS_TIMER_H
