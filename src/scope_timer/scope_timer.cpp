#include <mrs_lib/scope_timer.h>

namespace mrs_lib
{

// | ------------------------ ScopeTimer ------------------------ |

/* ScopeTimer constructor //{ */

ScopeTimer::ScopeTimer(const std::string& label) {

  time_point point;
  point.ros_time    = ros::Time::now();
  point.chrono_time = std::chrono::system_clock::now();
  point.label       = "(timer start)";
  checkpoints.push_back(point);

  _timer_label_ = label;

  ROS_DEBUG("[%s]Scope timer started, label: %s", ros::this_node::getName().c_str(), label.c_str());
}

//}

/* checkpoint() //{ */

void ScopeTimer::checkpoint(const std::string& label) {

  time_point point;
  point.ros_time    = ros::Time::now();
  point.chrono_time = std::chrono::system_clock::now();
  point.label       = "(" + label + ")";
  checkpoints.push_back(point);
}

//}

/* ScopeTimer destructor //{ */

ScopeTimer::~ScopeTimer() {

  auto chrono_end_time = std::chrono::system_clock::now();
  auto ros_end_time    = ros::Time::now();

  time_point point;
  point.ros_time    = ros_end_time;
  point.chrono_time = chrono_end_time;
  point.label       = "(scope end)";
  checkpoints.push_back(point);

  int gap1 = 8;
  int gap2 = 8;
  int gap3 = 12;

  std::stringstream ss;
  ss.precision(3);
  ss << std::fixed;

  char separator = ' ';

  int width_label = 10;

  for (unsigned long i = 1; i < checkpoints.size(); i++) {
    int len = (checkpoints.at(i).label + checkpoints.at(i - 1).label).length();
    if (len > width_label) {
      width_label = len;
    }
  }
  width_label += 4;

  ss << std::endl << std::left << std::setw(width_label) << std::setfill(separator) << " {label}";
  ss << std::left << std::setw(gap1) << std::setfill(separator) << "";
  ss << std::left << std::setw(gap2) << std::setfill(separator) << "{ROS time}";
  ss << std::left << std::setw(gap3) << std::setfill(separator) << "";
  ss << std::left << std::setw(gap2) << std::setfill(separator) << "{chrono time}" << std::endl;

  for (unsigned long i = 1; i < checkpoints.size(); i++) {

    double ros_elapsed = (checkpoints.at(i).ros_time - checkpoints.at(i - 1).ros_time).toSec();
    int    ros_secs    = int(ros_elapsed);
    double ros_msecs   = std::fmod(ros_elapsed * 1000, 1000);

    std::chrono::duration<double> chrono_elapsed = (checkpoints.at(i).chrono_time - checkpoints.at(i - 1).chrono_time);
    int                           chrono_secs    = int(chrono_elapsed.count());
    double                        chrono_msecs   = std::fmod(chrono_elapsed.count() * 1000, 1000);

    ss << std::left << std::setw(width_label) << std::setfill(separator) << (checkpoints.at(i - 1).label + " -> " + checkpoints.at(i).label).c_str();

    ss << std::right << std::setw(gap1) << std::setfill(separator) << ros_secs << std::setw(0) << "s";
    ss << std::right << std::setw(gap2) << std::setfill(separator) << ros_msecs << std::setw(0) << "ms";
    ss << std::right << std::setw(gap3) << std::setfill(separator) << chrono_secs << std::setw(0) << "s";
    ss << std::right << std::setw(gap2) << std::setfill(separator) << chrono_msecs << std::setw(0) << "ms" << std::endl;
  }

  double ros_elapsed = (ros_end_time - checkpoints.at(0).ros_time).toSec();
  int    ros_secs    = int(ros_elapsed);
  double ros_msecs   = std::fmod(ros_elapsed * 1000, 1000);

  std::chrono::duration<double> chrono_elapsed = (chrono_end_time - checkpoints.at(0).chrono_time);
  int                           chrono_secs    = int(chrono_elapsed.count());
  double                        chrono_msecs   = std::fmod(chrono_elapsed.count() * 1000, 1000);

  ss << std::left << std::setw(width_label) << std::setfill(separator) << "TOTAL TIME";
  ss << std::right << std::setw(gap1) << std::setfill(separator) << ros_secs << std::setw(0) << "s";
  ss << std::right << std::setw(gap2) << std::setfill(separator) << ros_msecs << std::setw(0) << "ms";
  ss << std::right << std::setw(gap3) << std::setfill(separator) << chrono_secs << std::setw(0) << "s";
  ss << std::right << std::setw(gap2) << std::setfill(separator) << chrono_msecs << std::setw(0) << "ms" << std::endl;

  ROS_INFO("[%s]: Scope timer [%s] finished!%s", ros::this_node::getName().c_str(), _timer_label_.c_str(), ss.str().c_str());
}

//}

}  // namespace mrs_lib
