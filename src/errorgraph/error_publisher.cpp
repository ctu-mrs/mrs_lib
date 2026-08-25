#include <mrs_lib/errorgraph/error_publisher.h>

#include <map>

#if USE_ROS_TIMER == 1
using TimerType = mrs_lib::ROSTimer;
#else
using TimerType = mrs_lib::ThreadTimer;
#endif

namespace mrs_lib
{
  namespace errorgraph
  {

    ErrorPublisher::ErrorPublisher(const rclcpp::Node::SharedPtr node, const rclcpp::Clock::SharedPtr clock, const std::string& node_name,
                                   const std::string& component_name, const rclcpp::Rate& publish_period)
        : node_(node), clock_(clock), node_name_(node_name), component_name_(component_name)
    {
      known_components_.insert(component_name_);

      publisher_ = node_->create_publisher<mrs_msgs::msg::ErrorgraphElement>("~/errors", 32);

      cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      mrs_lib::TimerHandlerOptions timer_opts_start;

      timer_opts_start.node = node_;
      timer_opts_start.autostart = true;
      timer_opts_start.callback_group = cbkgrp_timers_;

      {
        auto callback_fcn = [this]() { publishErrors(); };

        timer_publisher_ = std::make_unique<TimerType>(timer_opts_start, publish_period, callback_fcn);
      }
    };

    void ErrorPublisher::flushAndShutdown()
    {
      publishErrors(/*skip_empty_heartbeats=*/true);
      timer_publisher_ = {};
      clock_->sleep_for(std::chrono::duration<double>(1.0));
      rclcpp::shutdown();
      exit(1);
    }

    void ErrorPublisher::addGeneralError(const error_id_t id, const std::string& description, const std::optional<std::string>& component_override)
    {
      const auto now = clock_->now();
      const std::string effective_component = component_override.value_or(component_name_);
      std::scoped_lock lck(errors_mtx_);
      known_components_.insert(effective_component);
      for (auto& error_wrapper : errors_)
      {
        if (error_wrapper.id.has_value() && error_wrapper.id.value() == id && error_wrapper.component == effective_component)
        {
          error_wrapper.msg.type = description;
          error_wrapper.msg.stamp = now;
          return;
        }
      }
      mrs_msgs::msg::ErrorgraphError msg;
      msg.type = description;
      msg.stamp = now;
      errors_.push_back({id, effective_component, std::move(msg)});
    }

    void ErrorPublisher::addOneshotError(const std::string& description, const std::optional<std::string>& component_override)
    {
      const auto now = clock_->now();
      const std::string effective_component = component_override.value_or(component_name_);
      std::scoped_lock lck(errors_mtx_);
      known_components_.insert(effective_component);
      mrs_msgs::msg::ErrorgraphError msg;
      msg.type = description;
      msg.stamp = now;
      errors_.push_back({std::nullopt, effective_component, std::move(msg)});
    }

    void ErrorPublisher::addWaitingForTopicError(const std::string& topic_name, const std::optional<std::string>& component_override)
    {
      const auto now = clock_->now();
      const std::string effective_component = component_override.value_or(component_name_);
      std::scoped_lock lck(errors_mtx_);
      known_components_.insert(effective_component);
      for (auto& error_wrapper : errors_)
      {
        if (error_wrapper.msg.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC && error_wrapper.msg.waited_for_topic == topic_name
            && error_wrapper.component == effective_component)
        {
          error_wrapper.msg.stamp = now;
          return;
        }
      }
      mrs_msgs::msg::ErrorgraphError msg;
      msg.type = mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC;
      msg.stamp = now;
      msg.waited_for_topic = topic_name;
      errors_.push_back({std::nullopt, effective_component, std::move(msg)});
    }

    void ErrorPublisher::addWaitingForTopicError(const std::string& topic_name, const node_id_t& expected_publisher,
                                                 const std::optional<std::string>& component_override)
    {
      const auto now = clock_->now();
      const std::string effective_component = component_override.value_or(component_name_);
      std::scoped_lock lck(errors_mtx_);
      known_components_.insert(effective_component);
      for (auto& error_wrapper : errors_)
      {
        if (error_wrapper.msg.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC && error_wrapper.msg.waited_for_topic == topic_name
            && error_wrapper.component == effective_component)
        {
          error_wrapper.msg.stamp = now;
          return;
        }
      }
      mrs_msgs::msg::ErrorgraphError msg;
      msg.type = mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_TOPIC;
      msg.stamp = now;
      msg.waited_for_topic = topic_name;
      msg.waited_for_node.node = expected_publisher.node;
      msg.waited_for_node.component = expected_publisher.component;
      errors_.push_back({std::nullopt, effective_component, std::move(msg)});
    }

    void ErrorPublisher::addWaitingForNodeError(const node_id_t& node_id, const std::optional<std::string>& component_override)
    {
      const auto now = clock_->now();
      const std::string effective_component = component_override.value_or(component_name_);
      std::scoped_lock lck(errors_mtx_);
      known_components_.insert(effective_component);
      for (auto& error_wrapper : errors_)
      {
        if (error_wrapper.msg.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE && error_wrapper.msg.waited_for_node.node == node_id.node
            && error_wrapper.msg.waited_for_node.component == node_id.component && error_wrapper.component == effective_component)
        {
          error_wrapper.msg.stamp = now;
          return;
        }
      }
      mrs_msgs::msg::ErrorgraphError msg;
      msg.type = mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE;
      msg.stamp = now;
      msg.waited_for_node.node = node_id.node;
      msg.waited_for_node.component = node_id.component;
      errors_.push_back({std::nullopt, effective_component, std::move(msg)});
    }

    void ErrorPublisher::publishErrors(bool skip_empty_heartbeats)
    {
      std::scoped_lock lck(errors_mtx_);
      const auto now = clock_->now();
      std::map<std::string, std::vector<mrs_msgs::msg::ErrorgraphError>> grouped;
      for (const auto& component : known_components_)
        grouped[component];
      for (const auto& error_wrapper : errors_)
        grouped[error_wrapper.component].push_back(error_wrapper.msg);
      for (auto& [component, errs] : grouped)
      {
        if (skip_empty_heartbeats && errs.empty())
          continue;
        mrs_msgs::msg::ErrorgraphElement msg;
        msg.stamp = now;
        msg.source_node.node = node_name_;
        msg.source_node.component = component;
        msg.errors = std::move(errs);
        publisher_->publish(msg);
      }
      errors_.clear();
    }
  } // namespace errorgraph
} // namespace mrs_lib
