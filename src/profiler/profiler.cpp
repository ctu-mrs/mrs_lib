#include <mrs_lib/Profiler.h>

namespace mrs_lib
{

Profiler::Profiler(ros::NodeHandle &nh_, std::string node_name) {

  publisher = nh_.advertise<mrs_msgs::ProfilerUpdate>("profiler", 100, false);

  this->node_name = node_name;

  mutex_publisher.unlock();

  ROS_INFO("[%s]: profiler initialized", node_name.c_str());
}

Routine *Profiler::registerRoutine(std::string name) {

  Routine *new_routine = new Routine(name, this->node_name, mutex_publisher, publisher);

  ROS_INFO("[%s]: new profiler routine registered (%s)", node_name.c_str(), name.c_str());

  return new_routine;
}

Routine *Profiler::registerRoutine(std::string name, int expected_rate, double threshold) {

  Routine *new_routine = new Routine(name, this->node_name, expected_rate, threshold, mutex_publisher, publisher);

  ROS_INFO("[%s]: new profiler routine registered (%s)", node_name.c_str(), name.c_str());

  return new_routine;
}

// constructor
Routine::Routine(std::string name, std::string node_name, int expected_rate, double threshold, std::mutex &mutex, ros::Publisher &publisher) {

  this->name           = name;
  this->node_name      = node_name;
  this->is_periodic_   = true;
  this->threshold_     = threshold;
  this->expected_rate_ = expected_rate;
  this->mutex          = &mutex;
  this->publisher      = &publisher;
}

// constructor
Routine::Routine(std::string name, std::string node_name, std::mutex &mutex, ros::Publisher &publisher) {

  this->name           = name;
  this->node_name      = node_name;
  this->is_periodic_   = false;
  this->expected_rate_ = 0;
  this->mutex          = &mutex;
  this->publisher      = &publisher;
}

void Routine::start(void) {
  this->execution_start = ros::Time::now();
}

void Routine::start(const ros::TimerEvent &event) {

  this->expected_start = event.current_expected;
  this->real_start     = event.current_real;

  this->execution_start = ros::Time::now();
}

void Routine::end(void) {
  this->execution_end = ros::Time::now();

  mrs_msgs::ProfilerUpdate new_update;

  new_update.stamp       = ros::Time::now();
  new_update.duration    = (execution_end - execution_start).toSec();
  new_update.is_periodic = this->is_periodic_;
  new_update.name        = this->name;
  new_update.iteration   = this->iteration++;
  if (this->is_periodic_) {
    new_update.expected_start = this->expected_start.toSec();
    new_update.real_start     = this->real_start.toSec();
    new_update.expected_rate  = this->expected_rate_;
  }

  mutex->lock();
  { publisher->publish(new_update); }
  mutex->unlock();
}
}
