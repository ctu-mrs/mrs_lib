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
  /**
   * @brief the basic constructor with a user-defined label of the timer
   */
  ScopeTimer(std::string label);

  /**
   * @brief checkpoint, prints the time passed until the point this function is called
   */
  void checkpoint(std::string label);

  /**
   * @brief the basic destructor
   */
  ~ScopeTimer();

private:
  struct time_point
  {
    ros::Time                                          ros_time;
    std::chrono::time_point<std::chrono::system_clock> chrono_time;
    std::string                                        label;
  };

  std::string             _timer_label_;
  std::vector<time_point> checkpoints;
};

}  // namespace mrs_lib

#endif
