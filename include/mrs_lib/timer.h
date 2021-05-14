#ifndef MRS_TIMER_H
#define MRS_TIMER_H

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace mrs_lib
{
  bool breakable_sleep(const ros::Duration& dur, const std::atomic<bool>& continue_sleep);

  /**
   * @brief Common wrapper representing the functionality of the ros::Timer.
   *
   * The implementation can then use either ros::Timer (the ROSTimer class)
   * or threads and synchronization primitives from the C++ standard library
   * (the ThreadTimer class). Both these variants implement the same interface.
   *
   * @note Functionality of the two implementations differs in some details.
   */
  class MRSTimer
  {
    public:
    using callback_t = std::function<void(const ros::TimerEvent&)>;

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

    /**
     * @brief returns true if callbacks should be called
     *
     * @return true if timer is running
     */
    virtual bool running() = 0;

    virtual ~MRSTimer() = default;
    MRSTimer(const MRSTimer&) = default;
    MRSTimer(MRSTimer&&) = default;
    MRSTimer& operator=(const MRSTimer&) = default;
    MRSTimer& operator=(MRSTimer&&) = default;

    protected:
    MRSTimer() = default;
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
     * @brief Constructs the object.
     *
     * @tparam ObjectType
     * @param nh                            ROS node handle to be used for creating the underlying ros::Timer object.
     * @param rate                          rate at which the callback should be called.
     * @param ObjectType::*const callback   callback method to be called.
     * @param obj                           object for the method.
     * @param oneshot                       whether the callback should only be called once after starting.
     * @param autostart                     whether the timer should immediately start after construction.
     */
    template <class ObjectType>
    ROSTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
             const bool oneshot = false, const bool autostart = true);

    /**
     * @brief full constructor
     *
     * @tparam ObjectType
     * @param nh                            ROS node handle to be used for creating the underlying ros::Timer object.
     * @param duration                      desired callback period.
     * @param ObjectType::*const callback   callback method to be called.
     * @param obj                           object for the method.
     * @param oneshot                       whether the callback should only be called once after starting.
     * @param autostart                     whether the timer should immediately start after construction.
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

    /**
     * @brief returns true if callbacks should be called
     *
     * @return true if timer is running
     */
    virtual bool running() override;

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
     * @brief Constructs the object.
     *
     * @tparam ObjectType
     * @param nh                            ignored (used just for consistency with ROSTimer)
     * @param rate                          rate at which the callback should be called.
     * @param ObjectType::*const callback   callback method to be called.
     * @param obj                           object for the method.
     * @param oneshot                       whether the callback should only be called once after starting.
     * @param autostart                     whether the timer should immediately start after construction.
     */
    template <class ObjectType>
    ThreadTimer(const ros::NodeHandle& nh, const ros::Rate& rate, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
                const bool oneshot = false, const bool autostart = true);

    /**
     * @brief Constructs the object.
     *
     * @tparam ObjectType
     * @param nh                            ignored (used just for consistency with ROSTimer)
     * @param duration                      desired callback period.
     * @param ObjectType::*const callback   callback method to be called.
     * @param obj                           object for the method.
     * @param oneshot                       whether the callback should only be called once after starting.
     * @param autostart                     whether the timer should immediately start after construction.
     */
    template <class ObjectType>
    ThreadTimer(const ros::NodeHandle& nh, const ros::Duration& duration, void (ObjectType::*const callback)(const ros::TimerEvent&), ObjectType* const obj,
                bool oneshot = false, const bool autostart = true);

    /**
     * @brief stop the timer
     *
     * No more callbacks will be called after this method returns.
     */
    virtual void stop() override;

    /**
     * @brief start the timer
     *
     * The first callback will be called in now + period.
     *
     * @note If the timer is already started, nothing will change.
     */
    virtual void start() override;

    /**
     * @brief set the timer period/duration
     *
     * The new period will be applied after the next callback.
     *
     * @param duration the new desired callback period.
     * @param reset    ignored in this implementation.
     */
    virtual void setPeriod(const ros::Duration& duration, const bool reset = true) override;

    /**
     * @brief returns true if callbacks should be called
     *
     * @return true if timer is running
     */
    virtual bool running() override;

    /**
     * @brief stops the timer and then destroys the object
     *
     * No more callbacks will be called when the destructor is started.
     */
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
