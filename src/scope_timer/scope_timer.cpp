#include <mrs_lib/scope_timer.h>

namespace mrs_lib
{

// | ------------------------ ScopeTimer ------------------------ |

/* ScopeTimer constructors //{ */

ScopeTimer::ScopeTimer(std::string label) {

  _timer_label_       = label;
  _ros_start_time_    = ros::Time::now();
  _chrono_start_time_ = std::chrono::system_clock::now();
  ROS_INFO("----- [%s]%s: Scope timer started %s", ros::this_node::getName().c_str(), _timer_label_.c_str(), "-----");
}

ScopeTimer::ScopeTimer() : ScopeTimer("") {
}

//}

/* printTime() //{ */
void ScopeTimer::printTime() {
  ScopeTimer("");
}

void ScopeTimer::printTime(std::string label) {

  auto chrono_now_time = std::chrono::system_clock::now();
  auto ros_now_time    = ros::Time::now();

  std::chrono::duration<double> chrono_elapsed = chrono_now_time - _chrono_start_time_;
  ros::Duration                 ros_elapsed    = ros_now_time - _ros_start_time_;

  if (label == "") {
    label = "";
  } else {
    label = "[" + label + "]";
  }

  ROS_INFO("[%s]%s%s: Checkpoint time since scope start: %fs (chrono time); %fs (ROS time)", ros::this_node::getName().c_str(), _timer_label_.c_str(),
           label.c_str(), chrono_elapsed.count(), ros_elapsed.toSec());

  chrono_last_check_time_ = chrono_now_time;
  ros_last_check_time_    = ros_now_time;
}

//}

/* printTimeSinceLastCheck() //{ */

void ScopeTimer::printTimeSinceLastCheck() {
  printTimeSinceLastCheck("");
}

void ScopeTimer::printTimeSinceLastCheck(std::string label) {

  auto chrono_now_time = std::chrono::system_clock::now();
  auto ros_now_time    = ros::Time::now();

  std::chrono::duration<double> chrono_elapsed = chrono_now_time - chrono_last_check_time_;
  ros::Duration                 ros_elapsed    = ros_now_time - ros_last_check_time_;

  if (label == "") {
    label = "";
  } else {
    label = "[" + label + "]";
  }

  ROS_INFO("[%s]%s%s: Checkpoint time since last checkpoint: %fs (chrono time); %fs (ROS time)", ros::this_node::getName().c_str(), _timer_label_.c_str(),
           label.c_str(), chrono_elapsed.count(), ros_elapsed.toSec());

  chrono_last_check_time_ = chrono_now_time;
  ros_last_check_time_    = ros_now_time;
}

//}

/* ScopeTimer destructor //{ */

ScopeTimer::~ScopeTimer() {

  auto chrono_end_time = std::chrono::system_clock::now();
  auto ros_end_time    = ros::Time::now();

  std::chrono::duration<double> chrono_elapsed = chrono_end_time - _chrono_start_time_;
  ros::Duration                 ros_elapsed    = ros_end_time - _ros_start_time_;

  std::string label;

  if (_timer_label_ == "") {
    label = "";
  } else {
    label = "[" + _timer_label_ + "]";
  }

  ROS_INFO("----- [%s]%s: Scope end time: %fs (chrono time); %fs (ROS time) %s", ros::this_node::getName().c_str(), label.c_str(), chrono_elapsed.count(),
           ros_elapsed.toSec(), "-----");
}

//}


}  // namespace mrs_lib
