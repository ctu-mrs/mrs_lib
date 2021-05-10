#include <mrs_lib/scope_timer.h>

namespace mrs_lib
{

// | ------------------------ ScopeTimer ------------------------ |

std::unordered_map<std::string, ros::Time> ScopeTimer::last_print_times;

/* ScopeTimer constructor //{ */

ScopeTimer::ScopeTimer(const std::string& label, const ros::Duration& throttle_period)
  : _timer_label_(label), _throttle_period_(throttle_period)
{
  checkpoints.push_back(time_point("timer start"));

  ROS_DEBUG("[%s]Scope timer started, label: %s", ros::this_node::getName().c_str(), label.c_str());
}

ScopeTimer::ScopeTimer(const std::string& label, const time_point& tp0, const ros::Duration& throttle_period)
  : _timer_label_(label), _throttle_period_(throttle_period)
{
  checkpoints.push_back(tp0);
  checkpoints.push_back(time_point("timer start"));

  ROS_DEBUG("[%s]Scope timer started, label: %s", ros::this_node::getName().c_str(), label.c_str());
}

//}

/* checkpoint() //{ */

void ScopeTimer::checkpoint(const std::string& label)
{
  checkpoints.push_back(time_point(label));
}

//}

/* ScopeTimer destructor //{ */

ScopeTimer::~ScopeTimer() {

  const auto chrono_end_time = std::chrono::system_clock::now();
  const auto ros_end_time    = ros::Time::now();

  // if throttling is enabled, check time of last print and only print if applicable
  if (!_throttle_period_.isZero())
  {
    bool do_print = false;
    const auto last_it = last_print_times.find(_timer_label_);
    // if this is the first print of this ScopeTimer
    if (last_it == last_print_times.end())
    {
      do_print = true;
      last_print_times.emplace(_timer_label_, ros_end_time);
    }
    else
    {
      // if this ScopeTimer was already printed, check how long ago
      ros::Time& last_print_time = last_it->second;
      if (ros_end_time - last_print_time > _throttle_period_)
      {
        // if it was long ago enough, print again and update the last print time
        do_print = true;
        last_print_time = ros_end_time;
      }
    }

    if (!do_print)
      return;
  }

  checkpoints.push_back(time_point("scope end"));

  int gap1 = 8;
  int gap2 = 8;
  int gap3 = 12;

  std::stringstream ss;
  ss.precision(3);
  ss << std::fixed;

  char separator = ' ';

  int max_label_width = 5;
  for (const auto& el : checkpoints)
  {
    const int len = el.label.length();
    if (len > max_label_width)
      max_label_width = len;
  }

  int width_label_column = 10;
  for (unsigned long i = 1; i < checkpoints.size(); i++) {
    const int len = (checkpoints.at(i).label + checkpoints.at(i - 1).label).length();
    if (len > width_label_column)
      width_label_column = len;
  }
  width_label_column += 7;

  ss << std::endl << std::left << std::setw(width_label_column) << std::setfill(separator) << " {label}";
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

    ss << std::left << std::setw(max_label_width) << std::setfill(separator) << checkpoints.at(i - 1).label;
    ss << " -> ";
    ss << std::left << std::setw(max_label_width) << std::setfill(separator) << checkpoints.at(i).label;

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

  ss << std::left << std::setw(width_label_column) << std::setfill(separator) << "TOTAL TIME";
  ss << std::right << std::setw(gap1) << std::setfill(separator) << ros_secs << std::setw(0) << "s";
  ss << std::right << std::setw(gap2) << std::setfill(separator) << ros_msecs << std::setw(0) << "ms";
  ss << std::right << std::setw(gap3) << std::setfill(separator) << chrono_secs << std::setw(0) << "s";
  ss << std::right << std::setw(gap2) << std::setfill(separator) << chrono_msecs << std::setw(0) << "ms" << std::endl;

  ROS_INFO("[%s]: Scope timer [%s] finished!%s", ros::this_node::getName().c_str(), _timer_label_.c_str(), ss.str().c_str());
}

//}

}  // namespace mrs_lib
