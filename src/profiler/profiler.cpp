#include <mrs_lib/profiler.h>

namespace mrs_lib
{

// | ------------------------ Profiler ------------------------ |

/* Profiler constructor //{ */

Profiler::Profiler() {
}

Profiler::Profiler(const rclcpp::Node::SharedPtr& node, const std::string& node_name, const bool profiler_enabled) {

  this->node_              = node;
  this->_node_name_        = node_name;
  this->_profiler_enabled_ = profiler_enabled;

  if (profiler_enabled) {
    publisher_ = std::make_shared<mrs_lib::PublisherHandler<mrs_msgs::msg::ProfilerUpdate>>(node_, "profiler");
  }

  RCLCPP_INFO(node_->get_logger(), "profiler initialized");

  this->is_initialized_ = true;
}

Profiler::Profiler(const Profiler& other) {

  this->is_initialized_    = other.is_initialized_;
  this->node_              = other.node_;
  this->_node_name_        = other._node_name_;
  this->_profiler_enabled_ = other._profiler_enabled_;

  if (this->_profiler_enabled_ && this->is_initialized_) {
    publisher_ = other.publisher_;
  }
}

Profiler& Profiler::operator=(const Profiler& other) {

  if (this == &other) {
    return *this;
  }

  this->is_initialized_    = other.is_initialized_;
  this->node_              = other.node_;
  this->_node_name_        = other._node_name_;
  this->_profiler_enabled_ = other._profiler_enabled_;

  if (this->_profiler_enabled_ && this->is_initialized_) {
    publisher_ = other.publisher_;
  }

  return *this;
}

//}

/* Profiler::registerRoutine() normal //{ */

Routine Profiler::createRoutine(const std::string& name) {

  return Routine(node_, name, this->_node_name_, publisher_, _profiler_enabled_);
}

//}

// | ------------------------- Routine ------------------------ |

/* Routine constructor for normal //{ */

Routine::Routine(const rclcpp::Node::SharedPtr& node, const std::string& name, const std::string& node_name,
                 const std::shared_ptr<mrs_lib::PublisherHandler<mrs_msgs::msg::ProfilerUpdate>>& publisher, bool profiler_enabled) {

  if (!profiler_enabled) {
    return;
  }

  node_ = node;

  this->publisher_ = publisher;

  this->_routine_name_  = name;
  msg_out_.routine_name = name;

  this->_node_name_  = node_name;
  msg_out_.node_name = node_name;

  this->_profiler_enabled_ = profiler_enabled;

  auto clock = node->get_clock();

  msg_out_.stamp    = clock->now();
  msg_out_.duration = 0;
  msg_out_.event    = mrs_msgs::msg::ProfilerUpdate::START;

  execution_start_ = clock->now();

  publisher_->publish(msg_out_);
}

//}

/* end() //{ */

void Routine::end(void) {

  if (!_profiler_enabled_) {
    return;
  }

  auto clock = node_->get_clock();

  rclcpp::Time execution_end = clock->now();

  msg_out_.stamp    = execution_end;
  msg_out_.duration = (execution_end - execution_start_).seconds();

  msg_out_.event = mrs_msgs::msg::ProfilerUpdate::END;

  publisher_->publish(msg_out_);
}

//}

/* ~Routine() //{ */

Routine::~Routine() {

  this->end();
}

//}

}  // namespace mrs_lib
