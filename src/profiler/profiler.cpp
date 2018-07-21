#include <mrs_lib/Profiler.h>

namespace mrs_lib
{

Profiler::Profiler() {
}

Profiler::Profiler(ros::NodeHandle &nh_, std::string node_name) {

  publisher = nh_.advertise<mrs_msgs::ProfilerUpdate>("profiler", 100, false);

  this->node_name = node_name;

  ROS_INFO("[%s]: profiler initialized", node_name.c_str());
}

Routine *Profiler::registerRoutine(std::string name) {

  ROS_INFO("[%s]: new profiler routine registered (%s)", node_name.c_str(), name.c_str());

  Routine *new_routine = new Routine(name, this->node_name, publisher, mutex_publisher);

  return new_routine;
}

Routine *Profiler::registerRoutine(std::string name, int expected_rate, double threshold) {

  ROS_INFO("[%s]: new profiler routine registered (%s)", node_name.c_str(), name.c_str());

  Routine *new_routine = new Routine(name, this->node_name, expected_rate, threshold, publisher, mutex_publisher);

  return new_routine;
}

Routine::Routine(){};

Routine::Routine(std::string name, std::string node_name, int expected_rate, double threshold, ros::Publisher &publisher, std::mutex &mutex_publisher) {

  this->routine_name    = name;
  this->node_name       = node_name;
  this->is_periodic_    = true;
  this->threshold_      = threshold;
  this->expected_rate_  = expected_rate;
  this->publisher       = &publisher;
  this->mutex_publisher = &mutex_publisher;
}

// constructor
Routine::Routine(std::string name, std::string node_name, ros::Publisher &publisher, std::mutex &mutex_publisher) {

  this->routine_name    = name;
  this->node_name       = node_name;
  this->is_periodic_    = false;
  this->expected_rate_  = 0;
  this->publisher       = &publisher;
  this->mutex_publisher = &mutex_publisher;
}

void Routine::start(void) {
  this->execution_start = ros::Time::now();
}

void Routine::start(const ros::TimerEvent &event) {

  this->expected_start = event.current_expected;
  this->real_start     = event.current_real;

  double dt = (real_start - expected_start).toSec();

  if (dt > threshold_) {
    ROS_WARN("[%s]: routine '%s' was lauched late by %1.3f s!", node_name.c_str(), routine_name.c_str(), dt);
  }

  this->execution_start = ros::Time::now();
}

void Routine::end(void) {
  this->execution_end = ros::Time::now();

  mrs_msgs::ProfilerUpdate new_update;

  new_update.stamp        = ros::Time::now();
  new_update.duration     = (execution_end - execution_start).toSec();
  new_update.is_periodic  = this->is_periodic_;
  new_update.node_name    = this->node_name;
  new_update.routine_name = this->routine_name;
  new_update.iteration    = this->iteration++;

  if (this->is_periodic_) {
    new_update.expected_start = this->expected_start.toSec();
    new_update.real_start     = this->real_start.toSec();
    new_update.expected_rate  = this->expected_rate_;
  }

  mutex_publisher->lock();
  { publisher->publish(new_update); }
  mutex_publisher->unlock();
}
}
