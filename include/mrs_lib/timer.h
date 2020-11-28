#ifndef MRS_TIMER_H
#define MRS_TIMER_H

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <condition_variable>

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

  std::mutex mutex_timer_;

  std::shared_ptr<ros::Timer> timer_;
};

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
  class Impl;

  std::shared_ptr<Impl> impl_;

};  // namespace mrs_lib

//}

/* class ThreadTimer::Impl //{ */

class ThreadTimer::Impl {
public:
  Impl();
  ~Impl();

  void start(void);
  void stop(void);
  void setPeriod(const ros::Duration& duration, const bool& reset = true);

  bool          oneshot_;
  bool          shoot_ = false;
  ros::Rate     rate_;
  ros::Duration duration_;

  std::function<void(const ros::TimerEvent&)> callback_;

private:
  bool      running_;
  ros::Time next_expected_;
  ros::Time last_expected_;
  ros::Time last_real_;

  bool        thread_created_ = false;
  std::thread thread_;

  void threadFcn(void);

  bool       rate_changed_ = false;
  std::mutex mutex_time_;

  // for oneshot

  std::mutex mutex_oneshot_;
  std::condition_variable oneshot_cond_;
};

//}

#include <impl/timer.hpp>

}  // namespace mrs_lib

#endif  // MRS_TIMER_H
