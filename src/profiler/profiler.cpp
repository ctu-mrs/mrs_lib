#include <mrs_lib/Profiler.h>

namespace mrs_lib
{

// | ------------------------ Profiler ------------------------ |

/* Profiler constructor //{ */

Profiler::Profiler(ros::NodeHandle &nh_, std::string node_name, bool profiler_enabled) {

  this->node_name = node_name;

  this->profiler_enabled_ = profiler_enabled;

  if (profiler_enabled) {
    mutex_publisher = std::make_shared<std::mutex>();
    publisher = std::make_shared<ros::Publisher>(nh_.advertise<mrs_msgs::ProfilerUpdate>("profiler", 100, false));
  }

  ROS_INFO("[%s]: profiler initialized", node_name.c_str());
}

//}

/* Profiler::registerRoutine() for periodic //{ */

Routine Profiler::createRoutine(std::string name, int expected_rate, double threshold, ros::TimerEvent event) {

  return Routine(name, this->node_name, expected_rate, threshold, publisher, mutex_publisher, profiler_enabled_, event);
}

//}

/* Profiler::registerRoutine() normal //{ */

Routine Profiler::createRoutine(std::string name) {

  return Routine(name, this->node_name, publisher, mutex_publisher, profiler_enabled_);
}

//}

// | ------------------------- Routine ------------------------ |

/* Routine constructor for periodic //{ */

Routine::Routine(std::string name, std::string node_name, int expected_rate, double threshold, std::shared_ptr<ros::Publisher> publisher,
                 std::shared_ptr<std::mutex> mutex_publisher, bool profiler_enabled, ros::TimerEvent event) {

  if (!profiler_enabled) {
    return;
  }

  threshold_ = threshold;

  this->publisher       = publisher;
  this->mutex_publisher = mutex_publisher;

  this->routine_name   = name;
  msg_out.routine_name = name;

  this->node_name   = node_name;
  msg_out.node_name = node_name;

  this->profiler_enabled_ = profiler_enabled;

  msg_out.is_periodic   = true;
  msg_out.expected_rate = expected_rate;

  msg_out.expected_start = event.current_expected.toSec();
  msg_out.real_start     = event.current_real.toSec();

  msg_out.stamp     = ros::Time::now();
  msg_out.duration  = 0;
  msg_out.iteration = this->iteration++;
  msg_out.event     = mrs_msgs::ProfilerUpdate::START;

  execution_start = ros::Time::now();

  double dt = msg_out.real_start - msg_out.expected_start;

  if (dt > 0.01) {
    ROS_WARN_THROTTLE(1.0, "[%s]: routine '%s' was lauched late by %1.3f s!", node_name.c_str(), routine_name.c_str(), dt + threshold_);
  }

  {
    std::scoped_lock lock(*mutex_publisher);

    try {
      publisher->publish(mrs_msgs::ProfilerUpdateConstPtr(new mrs_msgs::ProfilerUpdate(msg_out)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher->getTopic().c_str());
    }
  }
}

//}

/* Routine constructor for normal //{ */

Routine::Routine(std::string name, std::string node_name, std::shared_ptr<ros::Publisher> publisher, std::shared_ptr<std::mutex> mutex_publisher, bool profiler_enabled) {

  if (!profiler_enabled) {
    return;
  }

  this->publisher       = publisher;
  this->mutex_publisher = mutex_publisher;

  this->routine_name   = name;
  msg_out.routine_name = name;

  this->node_name   = node_name;
  msg_out.node_name = node_name;

  this->profiler_enabled_ = profiler_enabled;

  msg_out.is_periodic   = false;
  msg_out.expected_rate = 0;

  msg_out.stamp      = ros::Time::now();
  msg_out.duration   = 0;
  msg_out.iteration  = this->iteration++;
  msg_out.event      = mrs_msgs::ProfilerUpdate::START;
  msg_out.real_start = msg_out.stamp.toSec();

  execution_start = ros::Time::now();

  {
    std::scoped_lock lock(*mutex_publisher);

    try {
      publisher->publish(mrs_msgs::ProfilerUpdateConstPtr(new mrs_msgs::ProfilerUpdate(msg_out)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher->getTopic().c_str());
    }
  }
}

//}

/* end() //{ */

void Routine::end(void) {

  if (!profiler_enabled_) {
    return;
  }

  ros::Time execution_end = ros::Time::now();

  msg_out.stamp    = ros::Time::now();
  msg_out.duration = (execution_end - execution_start).toSec();

  msg_out.event = mrs_msgs::ProfilerUpdate::END;

  {
    std::scoped_lock lock(*mutex_publisher);

    try {
      publisher->publish(mrs_msgs::ProfilerUpdateConstPtr(new mrs_msgs::ProfilerUpdate(msg_out)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher->getTopic().c_str());
    }
  }
}

//}

Routine::~Routine() {

  this->end();
}

}  // namespace mrs_lib
