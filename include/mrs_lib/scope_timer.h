/**  \file
 *   \brief A scoped function profiler which publishes ROS messages
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
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
   * @brief the basic destructor
   */
  ~ScopeTimer();

private:

  std::string                                        _label_string_;
  ros::Time                                          _ros_start_time_;
  std::chrono::time_point<std::chrono::system_clock> _chrono_start_time_;

};

}  // namespace mrs_lib

#endif
