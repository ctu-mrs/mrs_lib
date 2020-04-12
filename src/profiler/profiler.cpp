#include <mrs_lib/profiler.h>

namespace mrs_lib
{

// | ------------------------ Profiler ------------------------ |

/* Profiler constructor //{ */

Profiler::Profiler() {
}

Profiler::Profiler(ros::NodeHandle& nh, std::string _node_name_, bool profiler_enabled) {

  this->nh_                = std::make_shared<ros::NodeHandle>(nh);
  this->_node_name_        = _node_name_;
  this->_profiler_enabled_ = profiler_enabled;

  if (profiler_enabled) {
    mutex_publisher_ = std::make_unique<std::mutex>();
    publisher_       = std::make_unique<ros::Publisher>(this->nh_->advertise<mrs_msgs::ProfilerUpdate>("profiler", 100, false));
  }

  ROS_INFO("[%s]: profiler initialized", _node_name_.c_str());

  this->is_initialized_ = true;
}

Profiler::Profiler(const Profiler& other) {

  this->is_initialized_    = other.is_initialized_;
  this->nh_                = other.nh_;
  this->_node_name_        = other._node_name_;
  this->_profiler_enabled_ = other._profiler_enabled_;

  if (this->_profiler_enabled_ && this->is_initialized_) {
    mutex_publisher_ = std::make_unique<std::mutex>();
    publisher_       = std::make_unique<ros::Publisher>(this->nh_->advertise<mrs_msgs::ProfilerUpdate>("profiler", 100, false));
  }
}

Profiler& Profiler::operator=(const Profiler& other) {

  if (this == &other) {
    return *this;
  }

  this->is_initialized_    = other.is_initialized_;
  this->nh_                = other.nh_;
  this->_node_name_        = other._node_name_;
  this->_profiler_enabled_ = other._profiler_enabled_;

  if (this->_profiler_enabled_ && this->is_initialized_) {
    mutex_publisher_ = std::make_unique<std::mutex>();
    publisher_       = std::make_unique<ros::Publisher>(this->nh_->advertise<mrs_msgs::ProfilerUpdate>("profiler", 100, false));
  }

  return *this;
}

//}

/* Profiler::registerRoutine() for periodic //{ */

Routine Profiler::createRoutine(std::string name, double expected_rate, double threshold, ros::TimerEvent event) {

  return Routine(name, this->_node_name_, expected_rate, threshold, publisher_, mutex_publisher_, _profiler_enabled_, event);
}

//}

/* Profiler::registerRoutine() normal //{ */

Routine Profiler::createRoutine(std::string name) {

  return Routine(name, this->_node_name_, publisher_, mutex_publisher_, _profiler_enabled_);
}

//}

// | ------------------------- Routine ------------------------ |

/* Routine constructor for periodic //{ */

Routine::Routine(std::string name, std::string node_name, double expected_rate, double threshold, std::shared_ptr<ros::Publisher> publisher,
                 std::shared_ptr<std::mutex> mutex_publisher, bool profiler_enabled, ros::TimerEvent event) {

  if (!profiler_enabled) {
    return;
  }

  _threshold_ = threshold;

  this->publisher_       = publisher;
  this->mutex_publisher_ = mutex_publisher;

  this->_routine_name_  = name;
  msg_out_.routine_name = name;

  this->_node_name_  = node_name;
  msg_out_.node_name = node_name;

  this->_profiler_enabled_ = profiler_enabled;

  msg_out_.is_periodic   = true;
  msg_out_.expected_rate = expected_rate;

  msg_out_.expected_start = event.current_expected.toSec();
  msg_out_.real_start     = event.current_real.toSec();

  msg_out_.stamp     = ros::Time::now();
  msg_out_.duration  = 0;
  msg_out_.iteration = this->iteration_++;
  msg_out_.event     = mrs_msgs::ProfilerUpdate::START;

  execution_start_ = ros::Time::now();

  double dt = msg_out_.real_start - msg_out_.expected_start;

  if (dt > _threshold_) {
    ROS_WARN_THROTTLE(1.0, "[%s]: routine '%s' was lauched late by %.3f s!", _node_name_.c_str(), _routine_name_.c_str(), dt);
  }

  {
    std::scoped_lock lock(*mutex_publisher_);

    try {
      publisher_->publish(mrs_msgs::ProfilerUpdateConstPtr(new mrs_msgs::ProfilerUpdate(msg_out_)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_->getTopic().c_str());
    }
  }
}

//}

/* Routine constructor for normal //{ */

Routine::Routine(std::string name, std::string node_name, std::shared_ptr<ros::Publisher> publisher, std::shared_ptr<std::mutex> mutex_publisher,
                 bool profiler_enabled) {

  if (!profiler_enabled) {
    return;
  }

  this->publisher_       = publisher;
  this->mutex_publisher_ = mutex_publisher;

  this->_routine_name_  = name;
  msg_out_.routine_name = name;

  this->_node_name_  = node_name;
  msg_out_.node_name = node_name;

  this->_profiler_enabled_ = profiler_enabled;

  msg_out_.is_periodic   = false;
  msg_out_.expected_rate = 0;

  msg_out_.stamp      = ros::Time::now();
  msg_out_.duration   = 0;
  msg_out_.iteration  = this->iteration_++;
  msg_out_.event      = mrs_msgs::ProfilerUpdate::START;
  msg_out_.real_start = msg_out_.stamp.toSec();

  execution_start_ = ros::Time::now();

  {
    std::scoped_lock lock(*mutex_publisher_);

    try {
      publisher_->publish(mrs_msgs::ProfilerUpdateConstPtr(new mrs_msgs::ProfilerUpdate(msg_out_)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_->getTopic().c_str());
    }
  }
}

//}

/* end() //{ */

void Routine::end(void) {

  if (!_profiler_enabled_) {
    return;
  }

  ros::Time execution_end = ros::Time::now();

  msg_out_.stamp    = ros::Time::now();
  msg_out_.duration = (execution_end - execution_start_).toSec();

  msg_out_.event = mrs_msgs::ProfilerUpdate::END;

  {
    std::scoped_lock lock(*mutex_publisher_);

    try {
      publisher_->publish(mrs_msgs::ProfilerUpdateConstPtr(new mrs_msgs::ProfilerUpdate(msg_out_)));
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", publisher_->getTopic().c_str());
    }
  }
}

//}

Routine::~Routine() {

  this->end();
}

}  // namespace mrs_lib
