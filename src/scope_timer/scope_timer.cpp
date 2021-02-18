#include <mrs_lib/scope_timer.h>
#include <rclcpp/logger.hpp>

namespace mrs_lib
{

// instantiate the static member
rclcpp::Clock ScopeTimer::time_point::checkpoint_clock;

// | ------------------------ ScopeTimer ------------------------ |

/* ScopeTimer constructor //{ */

ScopeTimer::ScopeTimer(const rclcpp::Node& nh, const std::string& label) :
  m_logger(nh.get_logger()),
  m_timer_label(label)
{
  m_checkpoints.push_back(time_point("timer start"));
  RCLCPP_DEBUG(m_logger, "Scope timer started, label: %s");
}

ScopeTimer::ScopeTimer(const std::string& label) :
  m_logger(rclcpp::get_logger(label+" timer")),
  m_timer_label(label)
{
  m_checkpoints.push_back(time_point("timer start"));
  RCLCPP_DEBUG(m_logger, "Scope timer started, label: %s");
}

ScopeTimer::ScopeTimer(const std::string& label, const time_point& tp0) :
  m_logger(rclcpp::get_logger(label+" timer")),
  m_timer_label(label)
{
  m_checkpoints.push_back(tp0);
  m_checkpoints.push_back(time_point("timer start"));
  RCLCPP_DEBUG(m_logger, "Scope timer started, label: %s");
}

//}

/* checkpoint() //{ */

void ScopeTimer::checkpoint(const std::string& label)
{
  m_checkpoints.push_back(time_point(label));
}

//}

/* ScopeTimer destructor //{ */

ScopeTimer::~ScopeTimer()
{
  m_checkpoints.push_back(time_point("scope end"));
  const auto chrono_end_time = m_checkpoints.back().chrono_time;
  const auto ros_end_time = m_checkpoints.back().ros_time;

  int gap1 = 8;
  int gap2 = 8;
  int gap3 = 12;

  std::stringstream ss;
  ss.precision(3);
  ss << std::fixed;

  char separator = ' ';

  int max_label_width = 5;
  for (const auto& el : m_checkpoints)
  {
    const int len = el.label.length();
    if (len > max_label_width)
      max_label_width = len;
  }

  int width_label_column = 10;
  for (unsigned long i = 1; i < m_checkpoints.size(); i++) {
    const int len = (m_checkpoints.at(i).label + m_checkpoints.at(i - 1).label).length();
    if (len > width_label_column)
      width_label_column = len;
  }
  width_label_column += 7;

  ss << std::endl << std::left << std::setw(width_label_column) << std::setfill(separator) << " {label}";
  ss << std::left << std::setw(gap1) << std::setfill(separator) << "";
  ss << std::left << std::setw(gap2) << std::setfill(separator) << "{ROS time}";
  ss << std::left << std::setw(gap3) << std::setfill(separator) << "";
  ss << std::left << std::setw(gap2) << std::setfill(separator) << "{chrono time}" << std::endl;

  for (unsigned long i = 1; i < m_checkpoints.size(); i++) {

    double ros_elapsed = (m_checkpoints.at(i).ros_time - m_checkpoints.at(i - 1).ros_time).seconds();
    int    ros_secs    = int(ros_elapsed);
    double ros_msecs   = std::fmod(ros_elapsed * 1000, 1000);

    std::chrono::duration<double> chrono_elapsed = (m_checkpoints.at(i).chrono_time - m_checkpoints.at(i - 1).chrono_time);
    int                           chrono_secs    = int(chrono_elapsed.count());
    double                        chrono_msecs   = std::fmod(chrono_elapsed.count() * 1000, 1000);

    ss << std::left << std::setw(max_label_width) << std::setfill(separator) << m_checkpoints.at(i - 1).label;
    ss << " -> ";
    ss << std::left << std::setw(max_label_width) << std::setfill(separator) << m_checkpoints.at(i).label;

    ss << std::right << std::setw(gap1) << std::setfill(separator) << ros_secs << std::setw(0) << "s";
    ss << std::right << std::setw(gap2) << std::setfill(separator) << ros_msecs << std::setw(0) << "ms";
    ss << std::right << std::setw(gap3) << std::setfill(separator) << chrono_secs << std::setw(0) << "s";
    ss << std::right << std::setw(gap2) << std::setfill(separator) << chrono_msecs << std::setw(0) << "ms" << std::endl;
  }

  double ros_elapsed = (ros_end_time - m_checkpoints.at(0).ros_time).seconds();
  int    ros_secs    = int(ros_elapsed);
  double ros_msecs   = std::fmod(ros_elapsed * 1000, 1000);

  std::chrono::duration<double> chrono_elapsed = (chrono_end_time - m_checkpoints.at(0).chrono_time);
  int                           chrono_secs    = int(chrono_elapsed.count());
  double                        chrono_msecs   = std::fmod(chrono_elapsed.count() * 1000, 1000);

  ss << std::left << std::setw(width_label_column) << std::setfill(separator) << "TOTAL TIME";
  ss << std::right << std::setw(gap1) << std::setfill(separator) << ros_secs << std::setw(0) << "s";
  ss << std::right << std::setw(gap2) << std::setfill(separator) << ros_msecs << std::setw(0) << "ms";
  ss << std::right << std::setw(gap3) << std::setfill(separator) << chrono_secs << std::setw(0) << "s";
  ss << std::right << std::setw(gap2) << std::setfill(separator) << chrono_msecs << std::setw(0) << "ms" << std::endl;

  RCLCPP_INFO(m_logger, "Scope timer [%s] finished!%s", m_timer_label.c_str(), ss.str().c_str());
}

//}

}  // namespace mrs_lib
