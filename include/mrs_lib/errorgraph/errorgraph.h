#pragma once

#include <string>
#include <optional>
#include <vector>
#include <memory>
#include <algorithm>

#include <rclcpp/time.hpp>
#include "rclcpp/clock.hpp"

#include <mrs_lib/errorgraph/node_id.h>

#include <mrs_msgs/msg/errorgraph_error.hpp>
#include <mrs_msgs/msg/errorgraph_element.hpp>

namespace mrs_lib
{
  namespace errorgraph
  {
    using errorgraph_error_msg_t = mrs_msgs::msg::ErrorgraphError;
    using errorgraph_element_msg_t = mrs_msgs::msg::ErrorgraphElement;

    inline std::ostream& operator<<(std::ostream& os, const node_id_t& node_id)
    {
      return os << node_id.node << "." << node_id.component;
    }

    class Errorgraph
    {
    public:
      explicit Errorgraph(rclcpp::Clock::SharedPtr clock) : clock_(clock)
      {
      }
      ~Errorgraph() = default;

      /* error_t //{ */

      struct error_t
      {
        rclcpp::Time stamp; 
        std::string type;
        std::optional<std::string> waited_for_topic;
        std::optional<node_id_t> waited_for_node;

        inline bool is_waiting_for() const
        {
          return is_waiting_for_topic() || is_waiting_for_node();
        }

        inline bool is_waiting_for_topic() const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC;
        }

        inline bool is_waiting_for_node() const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE;
        }

        inline bool is_waiting_for(const std::string& topic) const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC && waited_for_topic.has_value() && waited_for_topic.value() == topic;
        }

        inline bool is_waiting_for(const node_id_t& node_id) const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE && waited_for_node.has_value() && waited_for_node.value() == node_id;
        }

        inline bool is_no_error() const
        {
          return type == errorgraph_error_msg_t::TYPE_NO_ERROR;
        }

        error_t(const errorgraph_error_msg_t& msg) : type(msg.type)
        {
          if (msg.type == errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE)
            waited_for_node = node_id_t::from_msg(msg.waited_for_node);
          else if (msg.type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC)
            waited_for_topic = msg.waited_for_topic;
          stamp = msg.stamp;
        }

        errorgraph_error_msg_t to_msg() const
        {
          errorgraph_error_msg_t ret;
          ret.stamp = stamp;
          ret.type = type;
          if (waited_for_topic.has_value())
            ret.waited_for_topic = waited_for_topic.value();
          if (waited_for_node.has_value())
            ret.waited_for_node = waited_for_node.value().to_msg();
          return ret;
        }
      };

      //}

      /* element_t //{ */

      struct element_t
      {
        enum class type_t
        {
          node,
          topic
        };

        size_t element_id;
        type_t type;

        // name of the topic (if this element is a topic)
        std::string topic_name;
        // name of this node (if this element is a node) or of the node
        // expected to publish this topic (if known)
        node_id_t source_node;

        // a list of errors related to this element (if it is a node)
        std::vector<error_t> errors;
        // last time this was updated from a message
        rclcpp::Time stamp;
        static constexpr double DEFAULT_NOT_REPORTING_DELAY_SECONDS = 3.0;
        rclcpp::Duration not_reporting_delay = rclcpp::Duration::from_seconds(DEFAULT_NOT_REPORTING_DELAY_SECONDS);

        // graph-related variables used by the build_graph() and find_dependency_roots() methods
        std::vector<element_t*> parents;
        std::vector<element_t*> children;
        bool visited = false;

        rclcpp::Clock::SharedPtr clock_;

        element_t(size_t element_id, const node_id_t& source_node, rclcpp::Clock::SharedPtr clock)
            : element_id(element_id), type(type_t::node), source_node(source_node), stamp(static_cast<int64_t>(0), clock->get_clock_type()), clock_(clock) {};

        element_t(size_t element_id, const std::string& topic_name, const node_id_t& source_node, rclcpp::Clock::SharedPtr clock)
            : element_id(element_id), type(type_t::topic), topic_name(topic_name), source_node(source_node),
            stamp(static_cast<int64_t>(0),clock->get_clock_type()), clock_(clock) {};

        inline std::vector<const std::string*> waited_for_topics() const
        {
          std::vector<const std::string*> ret;
          ret.reserve(errors.size());
          for (const auto& el : errors)
          {
            if (el.waited_for_topic.has_value())
              ret.push_back(&el.waited_for_topic.value());
          }
          return ret;
        }

        inline std::vector<const node_id_t*> waited_for_nodes() const
        {
          std::vector<const node_id_t*> ret;
          ret.reserve(errors.size());
          for (const auto& el : errors)
          {
            if (el.waited_for_node.has_value())
              ret.push_back(&el.waited_for_node.value());
          }
          return ret;
        }

        inline bool is_waiting_for() const
        {
          return std::any_of(std::begin(errors), std::end(errors), [](const auto& error) { return error.is_waiting_for(); });
        }

        inline bool is_waiting_for(const node_id_t& node_id) const
        {
          return std::any_of(std::begin(errors), std::end(errors), [node_id](const auto& error) { return error.is_waiting_for(node_id); });
        }

        inline bool is_no_error() const
        {
          return !is_not_reporting() && std::all_of(std::begin(errors), std::end(errors), [](const auto& error) { return error.is_no_error(); });
        }

        inline bool is_not_reporting() const
        {

          return type != type_t::topic && clock_->now() - stamp > not_reporting_delay;
        }

        errorgraph_element_msg_t to_msg() const
        {
          errorgraph_element_msg_t ret;
          ret.stamp = stamp;
          ret.source_node = source_node.to_msg();
          for (const auto& error : errors)
            ret.errors.push_back(error.to_msg());
          return ret;
        }
      };

      //}

    private:
      std::vector<std::unique_ptr<element_t>> elements_;
      bool graph_up_to_date_ = false;
      rclcpp::Clock::SharedPtr clock_;


      std::vector<element_t*> find_elements_waited_for(const element_t& element);

      element_t* find_element_mutable(const std::string& topic_name);

      element_t* find_element_mutable(const node_id_t& node_id);

      void prepare_graph();

      void build_graph();

      std::vector<const element_t*> DFS(element_t* from, bool* loop_detected_out = nullptr);

      element_t* add_new_element(const std::string& topic_name, const node_id_t& node_id = {});

      element_t* add_new_element(const node_id_t& node_id);

      size_t last_element_id = 0;

    public:
      void write_dot(std::ostream& os);

      std::vector<const element_t*> find_dependency_roots(const node_id_t& node_id, bool* loop_detected_out = nullptr);

      std::vector<const element_t*> find_error_roots();

      std::vector<const element_t*> find_roots();

      std::vector<const element_t*> find_leaves();

      const element_t* find_element(const std::string& topic_name);

      const element_t* find_element(const node_id_t& node_id);

      const element_t* add_element_from_msg(const errorgraph_element_msg_t& msg);

      auto begin() const
      {
        return elements_.begin();
      }
      auto end() const
      {
        return elements_.end();
      }
      auto cbegin() const
      {
        return elements_.cbegin();
      }
      auto cend() const
      {
        return elements_.cend();
      }
    };

  } // namespace errorgraph
} // namespace mrs_lib
