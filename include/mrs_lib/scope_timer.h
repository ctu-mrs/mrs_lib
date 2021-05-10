/**  \file
 *   \brief Simple timer which times a duration of its scope, with additional optional checkpoints.
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef SCOPE_TIMER_H
#define SCOPE_TIMER_H

#include <ros/ros.h>
#include <chrono>
#include <iostream>
#include <iomanip>
/* #include <ctime> */

namespace mrs_lib
{

/**
 * @brief simple timer which will time a duration of a scope and checkpoints inside the scope in ros time and std::chrono time
 */
class ScopeTimer {
public:
  /* time_point() helper struct //{ */
  struct time_point
  {
    using chrono_tp = std::chrono::time_point<std::chrono::system_clock>;
  
    public:
      time_point(const std::string& label) :
        ros_time(ros::Time::now()),
        chrono_time(std::chrono::system_clock::now()),
        label("(" + label + ")")
      {}
  
      time_point(const std::string& label, const ros::Time& ros_time) :
        ros_time(ros_time),
        label("(" + label + ")")
      {
        // helper types to make the code slightly more readable
        using float_seconds = std::chrono::duration<float>;
        using chrono_duration = std::chrono::system_clock::duration;
        // prepare ros and chrono current times to minimize their difference
        const auto ros_now = ros::Time::now();
        const auto chrono_now = std::chrono::system_clock::now();
        // calculate how old is ros_time
        const auto ros_dt = ros_now - ros_time;
        // cast ros_dt to chrono type
        const auto chrono_dt = std::chrono::duration_cast<chrono_duration>(float_seconds(ros_dt.toSec()));
        // calculate ros_time in chrono time and set it to chrono_time
        chrono_time = chrono_now - chrono_dt;
      }
  
      ros::Time ros_time;
      chrono_tp chrono_time;
      std::string label;
  };
  //}


public:
  /**
   * @brief the basic constructor with a user-defined label of the timer
   */
  ScopeTimer(const std::string& label, const ros::Duration& throttle_period = ros::Duration(0));

  /**
   * @brief the basic constructor with a user-defined label of the timer and a pre-start time, which will also be measured
   */
  ScopeTimer(const std::string& label, const time_point& tp0, const ros::Duration& throttle_period = ros::Duration(0));

  /**
   * @brief checkpoint, prints the time passed until the point this function is called
   */
  void checkpoint(const std::string& label);

  /**
   * @brief the basic destructor
   */
  ~ScopeTimer();

private:
  std::string             _timer_label_;
  ros::Duration           _throttle_period_;
  std::vector<time_point> checkpoints;

  static std::unordered_map<std::string, ros::Time> last_print_times;
};

}  // namespace mrs_lib

#endif
