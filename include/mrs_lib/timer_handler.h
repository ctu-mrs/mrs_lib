#ifndef MRS_TIMER_H
#define MRS_TIMER_H

#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include <mrs_lib/coro/runners.hpp>
#include <mrs_lib/coro/task.hpp>
#include <mrs_lib/internal/coroutine_callback_helpers.hpp>
#include <mrs_lib/utils.h>

namespace mrs_lib
{

  struct TimerHandlerOptions
  {
    TimerHandlerOptions(const rclcpp::Node::SharedPtr node) : node(node)
    {
    }

    TimerHandlerOptions() = default;

    rclcpp::Node::SharedPtr node;

    bool oneshot = false;

    bool autostart = true;

    std::optional<rclcpp::CallbackGroup::SharedPtr> callback_group;
  };

  /**
   * @brief Common wrapper representing the functionality of the rclcpp::Timer.
   *
   * The implementation can then use either rclcpp::Timer (the ROSTimer class)
   * or threads and synchronization primitives from the C++ standard library
   * (the ThreadTimer class). Both these variants implement the same interface.
   *
   * @note Functionality of the two implementations differs in some details.
   */
  class MRSTimer
  {
  public:
    using callback_t = std::function<void()>;

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
    virtual void setPeriod(const rclcpp::Duration& duration) = 0;

    /**
     * @brief change the callback method
     *
     * Usable e.g. for running thread with a specific parameter if you bind it using std::bind
     *
     * @param callback          callback method to be called.
     */
    virtual void setCallback(const std::function<void()>& callback) = 0;

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

    /**
     * @brief Create a callback for coroutine that should only run once at a time.
     *
     * Since coroutine callbacks are only allowed for reentrant groups,
     * the callback could be called while the previous is still in progress.
     * This helper function creates a callback that is skipped if the previous
     * one is still running.
     */
    template <typename C>
    static std::function<void()> createNonReentrantCallback(Task<> (C::*method)(), C* instance)
    {
      auto is_running = std::make_shared<std::atomic<bool>>(false);
      return [is_running, method, instance]() -> void {
        bool was_running = is_running->exchange(true);

        if (!was_running)
        {
          // The callback was not running and we have set it as running, we start it here.
          start_task(
              // Capturing lambdas should not be used as coroutines.
              // Pass the values as arguments instead.
              [](std::shared_ptr<std::atomic<bool>> is_running, Task<> (C::*method)(), C* instance) -> mrs_lib::Task<> {
                // Run the user specified callback.
                co_await std::invoke(method, instance);
                // The task finished so we store false to allow another callback to start.
                is_running->store(false);
                co_return;
              },
              is_running, method, instance);
        }
      };
    }

    rclcpp::Node::SharedPtr node_;
  };

  // | ------------------------ ROSTimer ------------------------ |

  /* class ROSTimer //{ */

  /**
   * @brief rclcpp::Timer wrapper. The interface is the same as with rclcpp::Timer, except for the initialization method.
   */
  class ROSTimer : public MRSTimer
  {
  public:
    ROSTimer();

    ROSTimer(const rclcpp::Node::SharedPtr& node, const rclcpp::Rate& rate, const std::function<void()>& callback);

    ROSTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, const std::function<void()>& callback);

    /**
     * @brief Create Ros timer with coroutine callback.
     *
     * Using coroutine callbacks is only allowed with reentrant callback group.
     * If the specified callback group is not reentrant, this throws an exception.
     *
     * To make the callback design simpler, the coroutine itself will not run
     * again until the previous one completely finished.
     */
    template <typename C>
    ROSTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, Task<> (C::*method)(), C* instance)
        : ROSTimer(opts, rate, createNonReentrantCallback(method, instance))
    {
      internal::require_callback_group_coro_compatible(opts.callback_group.value_or(nullptr));
    }

    /**
     * @brief Create Ros timer with coroutine callback.
     *
     * Using coroutine callbacks is only allowed with reentrant callback group.
     * If the specified callback group is not reentrant, this throws an exception.
     *
     * To make the callback design simpler, the coroutine itself will not run
     * again until the previous one completely finished.
     */
    template <typename C>
    ROSTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, Task<> (C::*method)(), std::shared_ptr<C> instance)
        : ROSTimer(opts, rate, createNonReentrantCallback(method, instance))
    {
      internal::require_callback_group_coro_compatible(opts.callback_group.value_or(nullptr));
    }

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
    virtual void setPeriod(const rclcpp::Duration& duration) override;

    /**
     * @brief change the callback method
     *
     * Usable e.g. for running thread with a specific parameter if you bind it using std::bind
     *
     * @param callback          callback method to be called.
     */
    virtual void setCallback(const std::function<void()>& callback) override;

    /**
     * @brief returns true if callbacks should be called
     *
     * @return true if timer is running
     */
    virtual bool running() override;

    virtual ~ROSTimer() override
    {
      stop();
    };

    // to keep rule of five since we have a custom destructor
    ROSTimer(const ROSTimer&) = delete;
    ROSTimer(ROSTimer&&) = delete;
    ROSTimer& operator=(const ROSTimer&) = delete;
    ROSTimer& operator=(ROSTimer&&) = delete;

  private:
    std::mutex mutex_timer_;

    std::function<void()> callback;

    bool oneshot;

    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<rclcpp::CallbackGroup::SharedPtr> callback_group_;
  };

  //}

  // | ----------------------- ThreadTimer ---------------------- |

  /* class ThreadTimer //{ */

  /**
   * @brief Custom thread-based Timers with the same interface as mrs_lib::ROSTimer.
   */
  class ThreadTimer : public MRSTimer
  {

  public:
    ThreadTimer();

    ThreadTimer(const rclcpp::Node::SharedPtr& node, const rclcpp::Rate& rate, const std::function<void()>& callback);

    ThreadTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, const std::function<void()>& callback);

    /**
     * @brief Create thread timer with coroutine callback.
     *
     * Using coroutine callbacks is only allowed with reentrant callback group.
     * If the specified callback group is not reentrant, this throws an exception.
     *
     * To make the callback design simpler, the coroutine itself will not run
     * again until the previous one completely finished.
     */
    template <typename C>
    ThreadTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, Task<> (C::*method)(), C* instance)
        : ThreadTimer(opts, rate, createNonReentrantCallback(method, instance))
    {
      internal::require_callback_group_coro_compatible(opts.callback_group.value_or(nullptr));
    }

    /**
     * @brief Create thread timer with coroutine callback.
     *
     * Using coroutine callbacks is only allowed with reentrant callback group.
     * If the specified callback group is not reentrant, this throws an exception.
     *
     * To make the callback design simpler, the coroutine itself will not run
     * again until the previous one completely finished.
     */
    template <typename C>
    ThreadTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, Task<> (C::*method)(), std::shared_ptr<C> instance)
        : ThreadTimer(opts, rate, createNonReentrantCallback(method, instance))
    {
      internal::require_callback_group_coro_compatible(opts.callback_group.value_or(nullptr));
    }

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
     */
    virtual void setPeriod(const rclcpp::Duration& duration) override;

    /**
     * @brief change the callback method
     *
     * Usable e.g. for running thread with a specific parameter if you bind it using std::bind
     *
     * @param callback          callback method to be called.
     */
    virtual void setCallback(const std::function<void()>& callback) override;

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
    virtual ~ThreadTimer() override
    {
      stop();
    };
    // to keep rule of five since we have a custom destructor
    ThreadTimer(const ThreadTimer&) = delete;
    ThreadTimer(ThreadTimer&&) = delete;
    ThreadTimer& operator=(const ThreadTimer&) = delete;
    ThreadTimer& operator=(ThreadTimer&&) = delete;

  private:
    class Impl;

    std::unique_ptr<Impl> impl_;

  }; // namespace mrs_lib

  //}

} // namespace mrs_lib

#ifndef MRS_TIMER_HPP
#include <mrs_lib/impl/timer_handler.hpp>
#endif

#endif // MRS_TIMER_H
