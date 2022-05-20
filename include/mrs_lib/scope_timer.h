/**  \file
 *   \brief Simple timer which times a duration of its scope, with additional optional checkpoints.
 *   \author Daniel Hert - hertdani@fel.cvut.cz
 */
#ifndef SCOPE_TIMER_H
#define SCOPE_TIMER_H

#include <ros/ros.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>
/* #include <ctime> */

namespace mrs_lib
{

/*//{ ScopeTimerLogger class */

/**
 * @brief Simple file logger of scope timer and its checkpoints
 */
class ScopeTimerLogger {

public:
  /**
   * @brief The basic constructor with a user-defined path to the logger file, enable flag and float-logging precision
   */
  ScopeTimerLogger(const std::string& logfile, const bool enable_logging = true, const int log_precision = 10);

  /**
   * @brief The basic destructor which closes the logging file
   */
  ~ScopeTimerLogger();

  /**
   * @brief Returns true if a non-empty path to a logger file was given by the user
   */
  bool shouldLog() {
    return _should_log_;
  }

  /**
   * @brief Returns true if the enable flag was set to true, a non-empty path to a logger file was given by the user and the logger file stream was successfully
   * opened.
   */
  bool loggingEnabled() {
    return _logging_enabled_;
  }

  /**
   * @brief Returns the path to the log.
   */
  std::string getLogFilepath() {
    return _log_filepath_;
  }

  using chrono_tp = std::chrono::time_point<std::chrono::steady_clock>;

  /**
   * @brief Writes the time data of the given scope and empty checkpoints into the logger stream.
   */
  void log(const std::string& scope, const chrono_tp& time_start, const chrono_tp& time_end);

  /**
   * @brief Writes the time data of the given scope and checkpoint labels into the logger stream.
   */
  void log(const std::string& scope, const std::string& label_from, const std::string& label_to, const chrono_tp& time_start, const chrono_tp& time_end);

private:
  bool          _logging_enabled_ = false;
  bool          _should_log_      = false;
  std::string   _log_filepath_;
  std::ofstream _logstream_;
  std::mutex    _mutex_logstream_;
};

/*//}*/

/**
 * @brief Simple timer which will time a duration of a scope and checkpoints inside the scope in ros time and std::chrono time.
 */
class ScopeTimer {
public:
  /* time_point() helper struct //{ */
  struct time_point
  {
    using chrono_tp = std::chrono::time_point<std::chrono::steady_clock>;

  public:
    time_point(const std::string& label) : ros_time(ros::Time::now()), chrono_time(std::chrono::steady_clock::now()), label(label) {
    }

    time_point(const std::string& label, const ros::Time& ros_time) : ros_time(ros_time), label(label) {
      // helper types to make the code slightly more readable
      using float_seconds   = std::chrono::duration<float>;
      using chrono_duration = std::chrono::steady_clock::duration;
      // prepare ros and chrono current times to minimize their difference
      const auto ros_now    = ros::Time::now();
      const auto chrono_now = std::chrono::steady_clock::now();
      // calculate how old is ros_time
      const auto ros_dt = ros_now - ros_time;
      // cast ros_dt to chrono type
      const auto chrono_dt = std::chrono::duration_cast<chrono_duration>(float_seconds(ros_dt.toSec()));
      // calculate ros_time in chrono time and set it to chrono_time
      chrono_time = chrono_now - chrono_dt;
    }

    ros::Time   ros_time;
    chrono_tp   chrono_time;
    std::string label;
  };
  //}

public:
  /**
   * @brief The basic constructor with a user-defined label of the timer, throttled period and file logger.
   */
  ScopeTimer(const std::string& label, const ros::Duration& throttle_period = ros::Duration(0), const bool enable = true,
             const std::shared_ptr<ScopeTimerLogger> scope_timer_logger = nullptr);

  /**
   * @brief The basic constructor with a user-defined label of the timer and a pre-start time, which will also be measured.
   */
  ScopeTimer(const std::string& label, const time_point& tp0, const ros::Duration& throttle_period = ros::Duration(0), const bool enable = true,
             const std::shared_ptr<ScopeTimerLogger> scope_timer_logger = nullptr);

  /**
   * @brief The basic constructor with a user-defined label of the timer and file logger.
   */
  ScopeTimer(const std::string& label, const std::shared_ptr<ScopeTimerLogger> scope_timer_logger, const bool enable = true);

  /**
   * @brief Checkpoint, prints the time passed until the point this function is called.
   */
  void checkpoint(const std::string& label);

  /**
   * @brief Getter for scope timer lifetime
   *
   * @return lifetime as floating point milliseconds
   */
  float getLifetime();

  /**
   * @brief The basic destructor which prints out or logs the scope times, if enabled.
   */
  ~ScopeTimer();

private:
  std::string             _timer_label_;
  bool                    _enable_print_or_log;
  ros::Duration           _throttle_period_;
  std::vector<time_point> checkpoints;

  std::shared_ptr<ScopeTimerLogger> _logger_ = nullptr;

  static std::unordered_map<std::string, ros::Time> last_print_times;
};

}  // namespace mrs_lib

#endif
