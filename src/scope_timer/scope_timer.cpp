#include <mrs_lib/scope_timer.h>

namespace mrs_lib
{

// | --------------------- ScopeTimerLogger --------------------- |

/*//{ ScopeTimerLogger constructor */
ScopeTimerLogger::ScopeTimerLogger(const rclcpp::Node::SharedPtr& node, const std::string& logfile, const bool enable_logging, const int log_precision)
    : _logging_enabled_(enable_logging), _log_filepath_(logfile) {

  node_ = node;

  if (!_logging_enabled_) {
    return;
  } else if (logfile.empty()) {
    _logging_enabled_ = false;
    RCLCPP_INFO(node_->get_logger(), "ScopeTimerLogger: Logfile path not specified. Skipping logging to file.");
    return;
  }
  _should_log_ = true;

  try {
    std::scoped_lock lock(_mutex_logstream_);

    _logstream_ = std::ofstream(logfile, std::ios_base::out | std::ios_base::trunc);

    if (!_logstream_.is_open()) {
      _logging_enabled_ = false;
      RCLCPP_ERROR(node_->get_logger(), "Scope timer failed to create log file with path (%s). Skipping logging.", logfile.c_str());
      return;
    }

    _logstream_ << std::fixed << std::setprecision(log_precision);
    _logstream_ << "#scope,label_from,label_to,sec_start,sec_end,sec_duration" << std::endl;
    _logging_enabled_ = true;
  }
  catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "Scope timer logger could not open logger file at: %s. Skipping logging.", logfile.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Scope timer logger path: %s.", logfile.c_str());
}
/*//}*/

/*//{ ScopeTimerLogger destructor */
ScopeTimerLogger::~ScopeTimerLogger() {
  if (_logging_enabled_) {
    RCLCPP_DEBUG(node_->get_logger(), "ScopeTimerLogger: closing logstream.");
    _logstream_.close();
  }
}
/*//}*/

/*//{ ScopeTimerLogger::log() */
void ScopeTimerLogger::log(const std::string& scope, const chrono_tp& time_start, const chrono_tp& time_end) {
  log(scope, "", "", time_start, time_end);
}

void ScopeTimerLogger::log(const std::string& scope, const std::string& label_from, const std::string& label_to, const chrono_tp& time_start,
                           const chrono_tp& time_end) {

  if (!_logging_enabled_) {
    return;
  }

  const std::chrono::duration<double> duration_start = std::chrono::duration_cast<std::chrono::duration<double>>(time_start.time_since_epoch());
  const std::chrono::duration<double> duration_end   = std::chrono::duration_cast<std::chrono::duration<double>>(time_end.time_since_epoch());
  const std::chrono::duration<double> duration_total = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);

  {
    std::scoped_lock lock(_mutex_logstream_);
    _logstream_ << scope.c_str() << "," << label_from.c_str() << "," << label_to.c_str() << "," << duration_start.count() << "," << duration_end.count() << ","
                << duration_total.count() << std::endl;
  }
}
/*//}*/

// | ------------------------ ScopeTimer ------------------------ |

std::unordered_map<std::string, rclcpp::Time> ScopeTimer::last_print_times;

/* ScopeTimer constructor //{ */

ScopeTimer::ScopeTimer(const rclcpp::Node::SharedPtr& node, const std::string& label, const rclcpp::Duration& throttle_period, const bool enable,
                       const std::shared_ptr<ScopeTimerLogger> scope_timer_logger)
    : _timer_label_(label), _enable_print_or_log(enable), _throttle_period_(throttle_period), _logger_(scope_timer_logger) {

  node_ = node;

  checkpoints.push_back(time_point(node_, "timer start"));

  RCLCPP_DEBUG(node_->get_logger(), "Scope timer started, label: %s", label.c_str());
}

ScopeTimer::ScopeTimer(const rclcpp::Node::SharedPtr& node, const std::string& label, const time_point& tp0, const rclcpp::Duration& throttle_period,
                       const bool enable, const std::shared_ptr<ScopeTimerLogger> scope_timer_logger)
    : _timer_label_(label), _enable_print_or_log(enable), _throttle_period_(throttle_period), _logger_(scope_timer_logger) {

  node_ = node;

  checkpoints.push_back(tp0);
  checkpoints.push_back(time_point(node_, "timer start"));

  RCLCPP_DEBUG(node_->get_logger(), "Scope timer started with file logger (label: %s).", label.c_str());
}

ScopeTimer::ScopeTimer(const rclcpp::Node::SharedPtr& node, const std::string& label, const std::shared_ptr<ScopeTimerLogger> scope_timer_logger,
                       const bool enable)
    : _timer_label_(label), _enable_print_or_log(enable), _throttle_period_(0, 0), _logger_(scope_timer_logger) {

  node_ = node;

  checkpoints.push_back(time_point(node_, "timer start"));

  RCLCPP_DEBUG(node_->get_logger(), "Scope timer started with file logger (label: %s).", label.c_str());
}

//}

/* ScopeTimer::checkpoint() //{ */

void ScopeTimer::checkpoint(const std::string& label) {

  if (_enable_print_or_log) {
    checkpoints.push_back(time_point(node_, label));
  }
}

//}

/* ScopeTimer::getLifetime() //{ */

float ScopeTimer::getLifetime() {
  const auto lifetime_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - checkpoints.at(0).chrono_time).count();
  return lifetime_us / 1000.0f;
}

//}

/* ScopeTimer destructor //{ */

ScopeTimer::~ScopeTimer() {

  if (!_enable_print_or_log) {
    return;
  }

  const auto chrono_end_time = std::chrono::steady_clock::now();
  const auto ros_end_time    = node_->get_clock()->now();

  // if throttling is enabled, check time of last print and only print if applicable
  if (_throttle_period_.seconds() >= 1e-6) {

    bool       do_print = false;
    const auto last_it  = last_print_times.find(_timer_label_);

    // if this is the first print of this ScopeTimer
    if (last_it == last_print_times.end()) {
      do_print = true;
      last_print_times.emplace(_timer_label_, ros_end_time);
    } else {
      // if this ScopeTimer was already printed, check how long ago
      rclcpp::Time& last_print_time = last_it->second;
      if (ros_end_time - last_print_time > _throttle_period_) {
        // if it was long ago enough, print again and update the last print time
        do_print        = true;
        last_print_time = ros_end_time;
      }
    }

    if (!do_print)
      return;
  }

  // If logger object exists and it should log (a path to a logger file was given by the user)
  if (_logger_ && _logger_->shouldLog()) {

    // Enabled, if user sets the enable flag to true, a path to a logger file is given, and the logging stream was successfully opened
    if (!_logger_->loggingEnabled()) {
      return;
    }

    // Log checkpoints, e.g., "SCOPE->checkpoint1;checkpoint1->checkpoint2"
    std::string label_from = "";
    for (size_t i = 1; i < checkpoints.size(); i++) {
      const auto& checkpoint = checkpoints.at(i);
      _logger_->log(_timer_label_, label_from, checkpoint.label, checkpoints.at(i - 1).chrono_time, checkpoint.chrono_time);
      label_from = checkpoint.label;
    }

    // Log entire scope from start to end
    _logger_->log(_timer_label_, checkpoints.at(0).chrono_time, chrono_end_time);

  } else {

    checkpoints.push_back(time_point(node_, "scope end"));

    const int gap1 = 8;
    const int gap2 = 8;
    const int gap3 = 12;

    std::stringstream ss;
    ss.precision(3);
    ss << std::fixed;

    const char separator = ' ';

    int max_label_width = 5;
    for (auto& el : checkpoints) {
      el.label      = "(" + el.label + ")";
      const int len = el.label.length();
      if (len > max_label_width)
        max_label_width = len;
    }

    int width_label_column = 10;
    for (unsigned long i = 1; i < checkpoints.size(); i++) {

      const int len = (checkpoints.at(i).label + checkpoints.at(i - 1).label).length();

      if (len > width_label_column) {
        width_label_column = len;
      }
    }

    width_label_column += 7;

    ss << std::endl << std::left << std::setw(width_label_column) << std::setfill(separator) << " {label}";
    ss << std::left << std::setw(gap1) << std::setfill(separator) << "";
    ss << std::left << std::setw(gap2) << std::setfill(separator) << "{ROS time}";
    ss << std::left << std::setw(gap3) << std::setfill(separator) << "";
    ss << std::left << std::setw(gap2) << std::setfill(separator) << "{chrono time}" << std::endl;

    for (unsigned long i = 1; i < checkpoints.size(); i++) {

      const double ros_elapsed = (checkpoints.at(i).ros_time - checkpoints.at(i - 1).ros_time).seconds();
      const int    ros_secs    = int(ros_elapsed);
      const double ros_msecs   = std::fmod(ros_elapsed * 1000, 1000);

      const std::chrono::duration<double> chrono_elapsed = (checkpoints.at(i).chrono_time - checkpoints.at(i - 1).chrono_time);
      const int                           chrono_secs    = int(chrono_elapsed.count());
      const double                        chrono_msecs   = std::fmod(chrono_elapsed.count() * 1000, 1000);

      ss << std::left << std::setw(max_label_width) << std::setfill(separator) << checkpoints.at(i - 1).label;
      ss << " -> ";
      ss << std::left << std::setw(max_label_width) << std::setfill(separator) << checkpoints.at(i).label;

      ss << std::right << std::setw(gap1) << std::setfill(separator) << ros_secs << std::setw(0) << "s";
      ss << std::right << std::setw(gap2) << std::setfill(separator) << ros_msecs << std::setw(0) << "ms";
      ss << std::right << std::setw(gap3) << std::setfill(separator) << chrono_secs << std::setw(0) << "s";
      ss << std::right << std::setw(gap2) << std::setfill(separator) << chrono_msecs << std::setw(0) << "ms" << std::endl;
    }

    const double ros_elapsed = (ros_end_time - checkpoints.at(0).ros_time).seconds();
    const int    ros_secs    = int(ros_elapsed);
    const double ros_msecs   = std::fmod(ros_elapsed * 1000, 1000);

    const std::chrono::duration<double> chrono_elapsed = (chrono_end_time - checkpoints.at(0).chrono_time);
    const int                           chrono_secs    = int(chrono_elapsed.count());
    const double                        chrono_msecs   = std::fmod(chrono_elapsed.count() * 1000, 1000);

    ss << std::left << std::setw(width_label_column) << std::setfill(separator) << "TOTAL TIME";
    ss << std::right << std::setw(gap1) << std::setfill(separator) << ros_secs << std::setw(0) << "s";
    ss << std::right << std::setw(gap2) << std::setfill(separator) << ros_msecs << std::setw(0) << "ms";
    ss << std::right << std::setw(gap3) << std::setfill(separator) << chrono_secs << std::setw(0) << "s";
    ss << std::right << std::setw(gap2) << std::setfill(separator) << chrono_msecs << std::setw(0) << "ms" << std::endl;

    RCLCPP_INFO(node_->get_logger(), "Scope timer [%s] finished!%s", _timer_label_.c_str(), ss.str().c_str());
  }
}

//}

}  // namespace mrs_lib
