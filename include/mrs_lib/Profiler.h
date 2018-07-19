#ifndef PROFILER_H
#define PROFILER_H

#include <ros/ros.h>
#include <mrs_msgs/ProfilerUpdate.h>
#include <mutex>

namespace mrs_lib
{

class Routine {

public:
  Routine();
  Routine(std::string name, std::string node_name, ros::Publisher &publisher, std::mutex &mutex_publisher);
  Routine(std::string name, std::string node_name, int expected_rate, double threshold, ros::Publisher &publisher, std::mutex &mutex_publisher);

  void start(const ros::TimerEvent &event);
  void start(void);
  void end(void);

private:
  std::string routine_name;
  std::string node_name;

  ros::Publisher *publisher;
  std::mutex *    mutex_publisher;

  // if periodic, those are the stats from the trigger event
  bool   is_periodic_ = false;
  double threshold_;
  int    expected_rate_ = 0;
  long   iteration      = 0;

  ros::Time expected_start;
  ros::Time real_start;

  // those are the stats from the execution of the routine
  ros::Time execution_start;
  ros::Time execution_end;
};

class Profiler {

public:
  Profiler();
  Profiler(ros::NodeHandle &nh_, std::string node_name);
  Routine *registerRoutine(std::string name, int expected_rate, double threshold);
  Routine *registerRoutine(std::string name);

private:
  ros::Publisher publisher;
  std::mutex     mutex_publisher;
  std::string    node_name;

private:
  std::vector<Routine> routine_list;
};
}

#endif
