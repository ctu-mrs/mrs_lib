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

/**
 * @brief ros::Timer wrapper. The interface is the same as with ros::Timer, except for the initialization method.
 */
class ROSTimer {
public:
  ROSTimer(void);
  ~ROSTimer(void);

  /**
   * @brief copy constructor
   *
   * @param other
   */
  ROSTimer(const ROSTimer& other);

  /**
   * @brief assignment operator
   *
   * @param other
   *
   * @return
   */
  ROSTimer& operator=(const ROSTimer& other);

  /**
   * @brief constructor, oneshot = false, autostart = true
   *
   * @tparam ObjectType
   * @param nh
   * @param rate
   * @param ObjectType::*const callback
   * @param obj
   */
  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj);

  /**
   * @brief constructor, oneshot = false, autostart = true
   *
   * @tparam ObjectType
   * @param nh
   * @param duration
   * @param ObjectType::*const callback
   * @param obj
   */
  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj);

  /**
   * @brief constructor, autostart = true
   *
   * @tparam ObjectType
   * @param nh
   * @param rate
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   */
  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot);

  /**
   * @brief constructor, autostart = true
   *
   * @tparam ObjectType
   * @param nh
   * @param duration
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   */
  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot);

  /**
   * @brief full constructor
   *
   * @tparam ObjectType
   * @param nh
   * @param rate
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   * @param autostart
   */
  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot, const bool& autostart);

  /**
   * @brief full constructor
   *
   * @tparam ObjectType
   * @param nh
   * @param duration
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   * @param autostart
   */
  template <class ObjectType>
  ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
           const bool& oneshot, const bool& autostart);

  /**
   * @brief stop the timer
   */
  void stop(void);

  /**
   * @brief start the timer
   */
  void start(void);

  /**
   * @brief set the timer period/duration
   *
   * @param duration
   * @param reset
   */
  void setPeriod(const ros::Duration& duration, const bool& reset = true);

private:
  std::mutex mutex_timer_;

  std::shared_ptr<ros::Timer> timer_;
};

//}

// | ----------------------- ThreadTimer ---------------------- |

/* class ThreadTimer //{ */

/**
 * @brief Custom thread-based Timers with the same interface as mrs_lib::ROSTimer.
 */
class ThreadTimer {

public:
  ThreadTimer();
  ~ThreadTimer();

  /**
   * @brief copy constructor
   *
   * @param other
   */
  ThreadTimer(const ThreadTimer& other);

  /**
   * @brief assignment operator
   *
   * @param other
   *
   * @return ThreadTimer
   */
  ThreadTimer& operator=(const ThreadTimer& other);

  /**
   * @brief constructor, oneshot = false, autostart = true
   *
   * @tparam ObjectType
   * @param nh node handle
   * @param rate ros::Rate
   * @param ObjectType::*const callback
   * @param obj
   */
  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj);

  /**
   * @brief constructor, oneshot = false, autostart = true
   *
   * @tparam ObjectType
   * @param nh
   * @param duration
   * @param ObjectType::*const callback
   * @param obj
   */
  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event),
              ObjectType* const obj);

  /**
   * @brief constructor, autostart = false
   *
   * @tparam ObjectType
   * @param nh
   * @param rate
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   */
  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot);

  /**
   * @brief constructor, autostart = false
   *
   * @tparam ObjectType
   * @param nh
   * @param duration
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   */
  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot);

  /**
   * @brief full constructor
   *
   * @tparam ObjectType
   * @param nh
   * @param rate
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   * @param autostart
   */
  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot, const bool& autostart);

  /**
   * @brief full constructor
   *
   * @tparam ObjectType
   * @param nh
   * @param duration
   * @param ObjectType::*const callback
   * @param obj
   * @param oneshot
   * @param autostart
   */
  template <class ObjectType>
  ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent& event), ObjectType* const obj,
              const bool& oneshot, const bool& autostart);

  /**
   * @brief stop the timer
   */
  void stop(void);

  /**
   * @brief start the timer
   */
  void start(void);

  /**
   * @brief set the timer period/duration
   *
   * @param duration
   * @param reset
   */
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

  std::mutex              mutex_oneshot_;
  std::condition_variable oneshot_cond_;
  bool                    oneshot_stop_waiting_ = false;

  bool       stop_oneshot_     = false;
  bool       prolong_oneshot_  = false;
  bool       oneshot_sleeping_ = false;
  ros::Time  prolong_to_;
  std::mutex mutex_prolong_;
};

//}

#include <impl/timer.hpp>

}  // namespace mrs_lib

#endif  // MRS_TIMER_H
