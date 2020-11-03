/**  \file
 *   \brief Simple timer which times a duration of its scope, with additional optional checkpoints.
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef SCOPE_TIMER_H
#define SCOPE_TIMER_H

#include <ros/ros.h>
#include <chrono>
/* #include <ctime> */

namespace mrs_lib
{

/**
 * @brief simple timer which will time a duration of a scope in ros time and std::chrono time
 */
class ScopeTimer {

public:

  /**
   * @brief the basic constructor
   */
  ScopeTimer();

  /**
   * @brief the basic constructor with a user-defined label of the timer
   */
  ScopeTimer(std::string label);

  /**
   * @brief checkpoint, prints the time passed until the point this function is called
   */
  void printTime();

  /**
   * @brief checkpoint, prints the time passed until the point this function is called. Inclueds a user defined label
   */
  void printTime(std::string label);

  /**
   * @brief checkpoint, prints the time passed since last time was printed.
   */
  void printTimeSinceLastCheck();

  /**
   * @brief checkpoint, prints the time passed since last time was printed.. Inclueds a user defined label
   */
  void printTimeSinceLastCheck(std::string label);
  /**
   * @brief the basic destructor
   */
  ~ScopeTimer();

private:

  std::string                                        _timer_label_;
  ros::Time                                          _ros_start_time_;
  std::chrono::time_point<std::chrono::system_clock> _chrono_start_time_;

  ros::Time                                          ros_last_check_time_;
  std::chrono::time_point<std::chrono::system_clock> chrono_last_check_time_;
};

}  // namespace mrs_lib

#endif
