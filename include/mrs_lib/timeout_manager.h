// clang: MatousFormat

/**  \file
     \brief TODO
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef TIMEOUT_MANAGER_H
#define TIMEOUT_MANAGER_H

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/timer_handler.h>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

namespace mrs_lib
{
  class TimeoutManager
  {
  public:
    // | ---------------------- public types ---------------------- |
    using timeout_id_t = size_t;
    using callback_t = std::function<void(const rclcpp::Time&)>;

  public:
    // | --------------------- public methods --------------------- |

    TimeoutManager(const std::shared_ptr<rclcpp::Node>& node, const rclcpp::Rate& update_rate);

    timeout_id_t registerNew(const rclcpp::Duration& timeout, const callback_t& callback, const rclcpp::Time& last_reset, const bool oneshot = false,
                             const bool autostart = true);

    void reset(const timeout_id_t id, const rclcpp::Time& time);

    void pause(const timeout_id_t id);

    void start(const timeout_id_t id, const rclcpp::Time& time);

    void pauseAll();

    void startAll();

    void change(const timeout_id_t id, const rclcpp::Duration& timeout, const callback_t& callback, const rclcpp::Time& last_reset, const bool oneshot = false,
                const bool autostart = true);

    rclcpp::Time lastReset(const timeout_id_t id);

    bool started(const timeout_id_t id);

    /* implementation details //{ */

  private:
    // | ---------------------- private types --------------------- |
    struct timeout_info_t
    {
      bool oneshot;
      bool started;
      callback_t callback;
      rclcpp::Duration timeout;
      rclcpp::Time last_reset;
      rclcpp::Time last_callback;
    };

  private:
    // | --------------------- private methods -------------------- |
    void main_timer_callback();

  private:
    // | ------------------------- members ------------------------ |
    std::recursive_mutex m_mtx;
    timeout_id_t m_last_id;
    std::vector<timeout_info_t> m_timeouts;

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::CallbackGroup::SharedPtr cb_grp_;

    std::shared_ptr<TimerType> m_main_timer;

    //}
  };

}  // namespace mrs_lib

#endif  // TIMEOUT_MANAGER_H
