#include <mrs_lib/scope_timer.h>

namespace mrs_lib
{

// | ------------------------ ScopeTimer ------------------------ |

/* ScopeTimer constructors //{ */

ScopeTimer::ScopeTimer(std::string label) {

  _label_string_      = label;
  _ros_start_time_    = ros::Time::now();
  _chrono_start_time_ = std::chrono::system_clock::now();
}

ScopeTimer::ScopeTimer() : ScopeTimer("") {
}

//}

/* ScopeTimer destructor //{ */

ScopeTimer::~ScopeTimer() {

  auto chrono_end_time = std::chrono::system_clock::now();
  auto ros_end_time    = ros::Time::now();

  std::chrono::duration<double> chrono_elapsed = chrono_end_time - _chrono_start_time_;
  ros::Duration                 ros_elapsed    = ros_end_time - _ros_start_time_;

  std::string label;

  if (_label_string_ == "") {
    label = "";
  } else {
    label = "[" + _label_string_ + "]";
  }

  ROS_INFO("[%s]%s: Scope time: %fs (chrono time); %fs (ROS time)", ros::this_node::getName().c_str(), label.c_str(), chrono_elapsed.count(), ros_elapsed.toSec());
}

//}


}  // namespace mrs_lib
