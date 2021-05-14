#ifndef MRS_TIMER_H
#define MRS_TIMER_H

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace mrs_lib
{

  class MRSTimer
  {
    public:

    MRSTimer() = default;

    /**
     * @brief stop the timer
     */
    virtual void stop() = 0;

    /**
     * @brief start the timer
     */
    virtual void start() = 0;

    /**
     * @brief set the timer period/duration
     *
     * @param duration
     * @param reset
     */
    virtual void setPeriod(const ros::Duration& duration, const bool reset = true) = 0;

    virtual ~MRSTimer() = default;
    MRSTimer(const MRSTimer&) = default;
    MRSTimer(MRSTimer&&) = default;
    MRSTimer& operator=(const MRSTimer&) = default;
    MRSTimer& operator=(MRSTimer&&) = default;
  };

  // | ------------------------ ROSTimer ------------------------ |

  /* class ROSTimer //{ */

  /**
   * @brief ros::Timer wrapper. The interface is the same as with ros::Timer, except for the initialization method.
   */
  class ROSTimer : public MRSTimer {
  public:
    ROSTimer();

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
    ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
             const bool oneshot = false, const bool autostart = true);

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
    ROSTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
             const bool oneshot = false, const bool autostart = true);

    /**
     * @brief stop the timer
     */
    virtual void stop() override;

    /**
     * @brief start the timer
     */
    virtual void start() override;

    /**
     * @brief set the timer period/duration
     *
     * @param duration
     * @param reset
     */
    virtual void setPeriod(const ros::Duration& duration, const bool reset = true) override;

    virtual ~ROSTimer() override {stop();};
    // to keep rule of five since we have a custom destructor
    ROSTimer(const ROSTimer&) = delete;
    ROSTimer(ROSTimer&&) = delete;
    ROSTimer& operator=(const ROSTimer&) = delete;
    ROSTimer& operator=(ROSTimer&&) = delete;

  private:
    std::mutex mutex_timer_;

    std::unique_ptr<ros::Timer> timer_;
  };

  //}

  // | ----------------------- ThreadTimer ---------------------- |

  /* class ThreadTimer //{ */

  /**
   * @brief Custom thread-based Timers with the same interface as mrs_lib::ROSTimer.
   */
  class ThreadTimer : public MRSTimer {

  public:
    ThreadTimer();

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
    ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
                const bool oneshot = false, const bool autostart = true);

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
    ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
                bool oneshot = false, const bool autostart = true);

    /**
     * @brief stop the timer
     */
    virtual void stop() override;

    /**
     * @brief start the timer
     */
    virtual void start() override;

    /**
     * @brief set the timer period/duration
     *
     * @param duration
     * @param reset
     */
    virtual void setPeriod(const ros::Duration& duration, const bool reset = true) override;

    virtual ~ThreadTimer() override {stop();};
    // to keep rule of five since we have a custom destructor
    ThreadTimer(const ThreadTimer&) = delete;
    ThreadTimer(ThreadTimer&&) = delete;
    ThreadTimer& operator=(const ThreadTimer&) = delete;
    ThreadTimer& operator=(ThreadTimer&&) = delete;

  private:
    class Impl;

    std::unique_ptr<Impl> impl_;

  };  // namespace mrs_lib

  //}

#include <impl/timer.hpp>

}  // namespace mrs_lib

#endif  // MRS_TIMER_H
