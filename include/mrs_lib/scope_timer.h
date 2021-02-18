/**  \file
 *   \brief Simple timer which times a duration of its scope, with additional optional checkpoints.
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef SCOPE_TIMER_H
#define SCOPE_TIMER_H

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>

namespace mrs_lib
{

/**
 * @brief simple timer which will time a duration of a scope and checkpoints inside the scope in ros time and std::chrono time
 */
class ScopeTimer {
public:
  struct time_point
  {
    private:
      using chrono_tp = std::chrono::time_point<std::chrono::system_clock>;
      static rclcpp::Clock checkpoint_clock;
    public:
      time_point(const std::string& label) :
        ros_time(checkpoint_clock.now()),
        chrono_time(std::chrono::system_clock::now()),
        label("(" + label + ")")
      {}

      time_point(const std::string& label, const rclcpp::Time& ros_time) :
        ros_time(ros_time),
        label("(" + label + ")")
      {
        // helper type to make the code slightly more readable
        using chrono_duration = std::chrono::system_clock::duration;
        // prepare ros and chrono current times to minimize their difference
        const auto ros_now = checkpoint_clock.now();
        const auto chrono_now = std::chrono::system_clock::now();
        // calculate how old is ros_time
        const auto ros_dt = ros_now - ros_time;
        // cast ros_dt to chrono type
        const auto chrono_dt = ros_dt.to_chrono<chrono_duration>();
        // calculate ros_time in chrono time and set it to chrono_time
        chrono_time = chrono_now - chrono_dt;
      }

      rclcpp::Time ros_time;
      chrono_tp chrono_time;
      std::string label;
  };


public:
  /**
   * @brief the basic constructor with a user-defined label of the timer and a node handle (will be used to get the logger)
   */
  ScopeTimer(const rclcpp::Node& nh, const std::string& label);

  /**
   * @brief the basic constructor with a user-defined label of the timer
   */
  ScopeTimer(const std::string& label);

  /**
   * @brief the basic constructor with a user-defined label of the timer and a pre-start time, which will also be measured
   */
  ScopeTimer(const std::string& label, const time_point& tp0);

  /**
   * @brief checkpoint, prints the time passed until the point this function is called
   */
  void checkpoint(const std::string& label);

  /**
   * @brief the basic destructor
   */
  ~ScopeTimer();

private:
  rclcpp::Logger          m_logger;
  const std::string       m_timer_label;
  std::vector<time_point> m_checkpoints;
};

}  // namespace mrs_lib

#endif
