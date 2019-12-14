/**  \file
 *   \brief A scoped function profiler which publishes ROS messages
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef PROFILER_H
#define PROFILER_H

#include <ros/ros.h>
#include <mrs_msgs/ProfilerUpdate.h>
#include <mutex>

namespace mrs_lib
{

class Routine {

public:
  Routine(std::string name, std::string node_name, std::shared_ptr<ros::Publisher> publisher, std::shared_ptr<std::mutex> mutex_publisher,
          bool profiler_enabled);
  Routine(std::string name, std::string node_name, double expected_rate, double threshold, std::shared_ptr<ros::Publisher> publisher,
          std::shared_ptr<std::mutex> mutex_publisher, bool profiler_enabled, ros::TimerEvent event);
  ~Routine();

  void end(void);

private:
  std::string _routine_name_;
  std::string _node_name_;

  std::shared_ptr<ros::Publisher> publisher_;
  std::shared_ptr<std::mutex>     mutex_publisher_;

  // if periodic, those are the stats from the trigger event
  double _threshold_;
  long   iteration_ = 0;

  bool _profiler_enabled_ = false;

  // those are the stats from the execution of the routine
  ros::Time execution_start_;

  // this will be published
  mrs_msgs::ProfilerUpdate msg_out_;
};

class Profiler {

public:
  /**
   * @brief the basic constructor
   */
  Profiler();

  /**
   * @brief the full constructor
   *
   * @param nh node handle
   * @param node_name the node name
   * @param profiler_enabled if profiling is enabled
   */
  Profiler(ros::NodeHandle& nh, std::string node_name, bool profiler_enabled);

  /**
   * @brief the copy constructor
   *
   * @param other the other object
   */
  Profiler(const Profiler& other);

  /**
   * @brief the assignment operator
   *
   * @param other the other object
   *
   * @return this object
   */
  Profiler& operator=(const Profiler& other);

  /**
   * @brief create a routine for a periodic function
   *
   * @param name the function name
   * @param expected_rate the expected rate in Hz
   * @param threshold the delay threshold to mark it as "late" in s
   * @param event the ros Timer event
   *
   * @return the Routine
   */
  Routine createRoutine(std::string name, double expected_rate, double threshold, ros::TimerEvent event);

  /**
   * @brief create a routine for an aperiodic function
   *
   * @param name the function name
   *
   * @return the Routine
   */
  Routine createRoutine(std::string name);

private:
  std::shared_ptr<ros::Publisher> publisher_;
  std::shared_ptr<std::mutex>     mutex_publisher_;
  std::string                     _node_name_;
  bool                            _profiler_enabled_ = false;

  std::shared_ptr<ros::NodeHandle> nh_;

  bool is_initialized_ = false;
};

}  // namespace mrs_lib

#endif
