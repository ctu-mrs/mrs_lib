#ifndef MRS_LIB_EXPERIMENTAL_TIMER_HPP_
#define MRS_LIB_EXPERIMENTAL_TIMER_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/timer.hpp>

#include "mrs_lib/utility/callback.hpp"
#include "utility/pimpl.hpp"

namespace mrs_lib
{

  /**
   * @brief ROS node interfaces required by the Timer.
   */
  using TimerNodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeTimersInterface,
                                                                      rclcpp::node_interfaces::NodeClockInterface>;

  namespace internal
  {

    /**
     * @brief Abstract interface for timer implementations.
     *
     * This interface allows the public Timer class to use different
     * timer implementations through the Pimpl idiom.
     */
    class TimerImplInterface
    {
    public:
      /**
       * @brief Default constructor.
       */
      TimerImplInterface() = default;

      /**
       * @brief Virtual destructor.
       */
      virtual ~TimerImplInterface() = default;

      TimerImplInterface(const TimerImplInterface&) = delete;
      TimerImplInterface& operator=(const TimerImplInterface&) = delete;
      TimerImplInterface(TimerImplInterface&&) = delete;
      TimerImplInterface& operator=(TimerImplInterface&&) = delete;

      /**
       * @brief Stop the timer.
       */
      virtual void stop() = 0;

      /**
       * @brief Start the timer or reset its countdown.
       */
      virtual void start_or_reset() = 0;

      /**
       * @brief Change the timer period.
       *
       * @param period New timer period.
       *
       * Implementations should reset the timer when the period is changed.
       */
      virtual void set_period(std::chrono::nanoseconds period) = 0;

      /**
       * @brief Check whether the timer is running.
       *
       * @return `true` if the timer is running, otherwise `false`.
       */
      [[nodiscard]] virtual bool is_running() const = 0;
    };

  } // namespace internal

  /**
   * @brief Options for creating a ROS-based timer.
   *
   * When configuring timer with these options, it will use rclcpp::Timer
   * internally.
   * (See mrs_lib::Timer documentation for differences in implementations.)
   */
  struct RosTimerOptions
  {
    /**
     * @brief ROS node interfaces used by the timer.
     */
    TimerNodeInterfaces node_interfaces;

    /**
     * @brief Whether the timer should start automatically after construction.
     */
    bool autostart = true;

    /**
     * @brief Whether the timer should execute its callback only once.
     *
     * If `true`, the timer stops after exactly one callback is executed.
     */
    bool oneshot = false;

    /**
     * @brief Callback group in which the timer callback should be executed.
     *
     * If `nullptr`, the default callback group is used.
     */
    std::shared_ptr<rclcpp::CallbackGroup> callback_group = nullptr;
  };

  /**
   * @brief Options for creating a thread-based timer.
   *
   * These options configure a timer that runs on a separate thread.
   * (See mrs_lib::Timer documentation for differences in implementations.)
   */
  struct ThreadTimerOptions
  {
    /**
     * @brief ROS node interfaces used by the timer.
     */
    TimerNodeInterfaces node_interfaces;

    /**
     * @brief Whether the timer should start automatically after construction.
     */
    bool autostart = true;

    /**
     * @brief Whether the timer should execute its callback only once.
     *
     * If `true`, the timer stops after exactly one callback is executed.
     */
    bool oneshot = false;
  };

  /**
   * @brief Timer wrapper supporting ROS and thread-based implementations.
   *
   * You can chose the timer implementation by calling the constructor with
   * the respective config type (@ref RosTimerOptions, @ref ThreadTimerOptions).
   *
   * @note
   * Although the thread timer implementation tries to be as close as possible
   * to the ROS timer, there are some important differences:
   * - *Clocks*
   *   - Ros timer uses the clock provided by the node interfaces.
   *   - Thread timer uses standard library clock (does not handle sim time!).
   * - *Callback Groups* - Thread timer does not handle rclcpp callback groups.
   *   This means that code relying on thread timers MUST manually ensure thread
   *   safe access to everything accessed by the timer callback.
   *   Using the ROS backend uses the callback groups as expected.
   * - *Callback Reentrancy* - Thread timer runs all callbacks on a single
   *   thread and thus is unable to start multiple callbacks in parallel like
   *   the ROS timer.
   */
  class Timer
  {
  public:
    /**
     * @brief Type of the accepted callback.
     */
    using EventCallback = std::function<void()>;

    /**
     * @brief Type of the accepted coroutine callback.
     */
    using EventCoroCallback = CoroCallback<void()>;

    /**
     * @brief Construct a ROS-based timer with a regular callback.
     *
     * @param options Timer configuration options.
     * @param period Timer period.
     * @param callback Callback invoked when the timer expires.
     */
    explicit Timer(const RosTimerOptions& options, std::chrono::nanoseconds period, EventCallback callback);

    /**
     * @brief Construct a ROS-based timer with a coroutine callback.
     *
     * @param options Timer configuration options.
     * @param period Timer period.
     * @param callback Coroutine callback invoked when the timer expires.
     */
    explicit Timer(const RosTimerOptions& options, std::chrono::nanoseconds period, EventCoroCallback callback);

    /**
     * @brief Construct a thread-based timer with a regular callback.
     *
     * @param options Timer configuration options.
     * @param period Timer period.
     * @param callback Callback invoked when the timer expires.
     */
    explicit Timer(const ThreadTimerOptions& options, std::chrono::nanoseconds period, EventCallback callback);

    /**
     * @brief Construct a thread-based timer with a coroutine callback.
     *
     * @param options Timer configuration options.
     * @param period Timer period.
     * @param callback Coroutine callback invoked when the timer expires.
     */
    explicit Timer(const ThreadTimerOptions& options, std::chrono::nanoseconds period, EventCoroCallback callback);

    /**
     * @brief Stop the timer.
     *
     * After calling this method, the timer will not invoke its callback until
     * @ref start_or_reset is called.
     */
    void stop();

    /**
     * @brief Start or reset the timer.
     *
     * If the timer is stopped, this starts it. If it is already running,
     * its countdown is reset using the currently configured period.
     */
    void start_or_reset();

    /**
     * @brief Set the timer period.
     *
     * @param period New timer period.
     *
     * Changing the period also resets the timer countdown and starts the timer
     * if it was previously stopped.
     */
    void set_period(std::chrono::nanoseconds period);

    /**
     * @brief Check whether the timer is currently running.
     *
     * @return `true` if the timer is running, otherwise `false`.
     */
    [[nodiscard]] bool is_running() const;

  private:
    Pimpl<internal::TimerImplInterface> impl_;
  };

} // namespace mrs_lib

#endif // MRS_LIB_EXPERIMENTAL_TIMER_HPP_
