#ifndef MRS_LIB_TIMEOUT_MANAGER_HPP_
#define MRS_LIB_TIMEOUT_MANAGER_HPP_


#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/time.hpp>

#include "mrs_lib/utility/owning_mutex.hpp"
#include "mrs_lib/utility/pimpl.hpp"


namespace mrs_lib
{
  class TimeoutManagerHandle;

  namespace internal
  {
    class TimeoutManagerHandleKey;
  }

  using TimeoutManagerNodeInterfaces =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface, rclcpp::node_interfaces::NodeTimersInterface,
                                              rclcpp::node_interfaces::NodeClockInterface>;


  /**
   * @brief Options for registering a timeout.
   */
  struct TimeoutManagerRegisterOptions
  {
    /** @brief Time of inactivity after which the callback is called. */
    std::chrono::nanoseconds timeout;
    /** @brief Callback called when timeout is triggered. */
    std::function<void(rclcpp::Time)> callback;
    /** @brief Whether the timeout should be created in the started state. */
    bool started = true;
    /** @brief Whether the timeout should stop after triggering once. */
    bool oneshot = false;
  };

  /**
   * @brief Manager for registering callbacks that are called after specified
   * time of inactivity.
   */
  class TimeoutManager2
  {
  private:
    class Impl;

  public:
    using Callback = std::function<void(rclcpp::Time)>;

    /**
     * @brief Construct the timeout manager.
     *
     * @param node_interfaces ROS node interfaces to use by the timeout manager.
     * @param update_period How often to check for the timeout expiration.
     */
    explicit TimeoutManager2(TimeoutManagerNodeInterfaces node_interfaces, std::chrono::nanoseconds update_period);

    /** @brief Destructor. */
    ~TimeoutManager2();

    /** @brief TimeoutManager2 is not copyable. */
    TimeoutManager2(const TimeoutManager2&) = delete;
    /** @brief TimeoutManager2 is not copyable. */
    TimeoutManager2& operator=(const TimeoutManager2&) = delete;

    /** @brief Move constructor. */
    TimeoutManager2(TimeoutManager2&&) noexcept;
    /** @brief Move assignment. */
    TimeoutManager2& operator=(TimeoutManager2&&) noexcept;

    /**
     * @brief Register a new timeout to be handled by this manager.
     *
     * @return RAII handle to the timeout for managing it.
     */
    std::shared_ptr<TimeoutManagerHandle> register_new(TimeoutManagerRegisterOptions options);

    /**
     * @brief Start all timeouts managed by this manager.
     */
    void start_all();

    /**
     * @brief Stop all timeouts managed by this manager.
     */
    void pause_all();

    /**
     * @brief Remove all timeouts for which the handles were already deleted.
     */
    void clear_deleted();

  private:
    Pimpl<Impl> impl_;

    friend class internal::TimeoutManagerHandleKey;
  };

  namespace internal
  {

    class TimeoutManagerHandleKey
    {
    private:
      TimeoutManagerHandleKey() = default;

      friend class TimeoutManager2::Impl;
    };

  } // namespace internal

  /**
   * @brief Handle to a registered timeout, that unregisters it on destruction.
   */
  class TimeoutManagerHandle
  {
  public:
    /**
     * @brief Construct the handle.
     *
     * @param key Key type to prevent users from calling this function.
     * @param options Options to configure the timeout.
     * @param clock_interface Clock interface to use for obtaining current time.
     *
     * @note This function is for internal use and should not be called from user code.
     */
    explicit TimeoutManagerHandle(internal::TimeoutManagerHandleKey key, TimeoutManagerRegisterOptions options,
                                  std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface);

    /** @brief Default destructor. */
    ~TimeoutManagerHandle() = default;
    /** @brief TimeoutManagerHandle is immovable. */
    TimeoutManagerHandle(const TimeoutManagerHandle&) = delete;
    /** @brief TimeoutManagerHandle is immovable. */
    TimeoutManagerHandle& operator=(const TimeoutManagerHandle&) = delete;
    /** @brief TimeoutManagerHandle is immovable. */
    TimeoutManagerHandle(TimeoutManagerHandle&&) = delete;
    /** @brief TimeoutManagerHandle is immovable. */
    TimeoutManagerHandle& operator=(TimeoutManagerHandle&&) = delete;

    /**
     * @brief Reset the timeout countdown.
     *
     * If this function is not called before the timeout expires,
     * the configured callback is called.
     */
    void reset_timeout();

    /**
     * @brief Start the timeout.
     */
    void start();

    /**
     * @brief Stop the timeout.
     *
     * After this call, new timeout callbacks won't be called until a call
     * to @ref start.
     */
    void pause();

    /**
     * @brief Set a new duration after which the timeout is called.
     */
    void set_timeout(std::chrono::nanoseconds timeout);

    /**
     * @brief Check if the timeout is started.
     *
     * @return `true` if it is started, `false` otherwise.
     */
    [[nodiscard]] bool is_started() const;

    /**
     * @brief Get time of last call to reset.
     *
     * @return Last time the @ref reset was called.
     */
    [[nodiscard]] rclcpp::Time get_last_reset() const;

    /**
     * @brief Check if timeout is expired and if yes, call it.
     *
     * @param key Key type to prevent users from calling this function.
     *
     * @note This function is for internal use and should not be called from user code.
     */
    void run_callback_if_expired(internal::TimeoutManagerHandleKey key);

  private:
    struct Data
    {
      std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface;
      TimeoutManagerRegisterOptions options;
      rclcpp::Time last_reset;
      rclcpp::Time last_callback;
    };

    OwningMutex<Data, std::recursive_mutex> data_;
  };

} // namespace mrs_lib

#endif // MRS_LIB_TIMEOUT_MANAGER_HPP_
