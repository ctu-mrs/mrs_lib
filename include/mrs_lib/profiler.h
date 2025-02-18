/**  \file
 *   \brief A scoped function profiler which publishes ROS messages
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef PROFILER_H
#define PROFILER_H

#include <rclcpp/rclcpp.hpp>
#include <mrs_msgs/msg/profiler_update.hpp>
#include <mrs_lib/publisher_handler.h>

namespace mrs_lib
{

class Routine {

public:
  Routine(const rclcpp::Node::SharedPtr& node, const std::string& name, const std::string& node_name,
          const std::shared_ptr<mrs_lib::PublisherHandler<mrs_msgs::msg::ProfilerUpdate>>& publisher, bool profiler_enabled);
  ~Routine();

  void end(void);

private:
  rclcpp::Node::SharedPtr node_;

  std::string _routine_name_;
  std::string _node_name_;

  std::shared_ptr<mrs_lib::PublisherHandler<mrs_msgs::msg::ProfilerUpdate>> publisher_;

  bool _profiler_enabled_ = false;

  // those are the stats from the execution of the routine
  rclcpp::Time execution_start_;

  // this will be published
  mrs_msgs::msg::ProfilerUpdate msg_out_;
};

class Profiler {

public:
  /**
   * @brief the basic constructor
   */
  Profiler();

  /**
   * @brief the full constructor
   *
   * @param node node handle
   * @param node_name the node name
   * @param profiler_enabled if profiling is enabled
   */
  Profiler(const rclcpp::Node::SharedPtr& node, const std::string& node_name, const bool profiler_enabled);

  /**
   * @brief the copy constructor
   *
   * @param other the other object
   */
  Profiler(const Profiler& other);

  /**
   * @brief the assignment operator
   *
   * @param other the other object
   *
   * @return this object
   */
  Profiler& operator=(const Profiler& other);

  /**
   * @brief create a routine for an aperiodic function
   *
   * @param name the function name
   *
   * @return the Routine
   */
  Routine createRoutine(const std::string& name);

private:
  std::shared_ptr<mrs_lib::PublisherHandler<mrs_msgs::msg::ProfilerUpdate>> publisher_;
  std::string                                                               _node_name_;
  bool                                                                      _profiler_enabled_ = false;

  rclcpp::Node::SharedPtr node_;

  bool is_initialized_ = false;
};

}  // namespace mrs_lib

#endif
