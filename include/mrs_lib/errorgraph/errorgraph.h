#pragma once

#include <string>
#include <optional>
#include <variant>
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

    /**
     * \brief A directed graph representing error dependencies between ROS nodes and topics.
     *
     * The Errorgraph aggregates error reports from nodes in the system (published as
     * \ref errorgraph_element_msg_t messages) and builds a dependency graph. Nodes report
     * which other nodes or topics they are waiting for, allowing the graph to trace error
     * propagation and identify root causes.
     *
     * Typical usage:
     * 1. Subscribe to error messages from all nodes in the system.
     * 2. Feed each received message to \ref add_element_from_msg().
     * 3. Query the graph using \ref find_error_roots(), \ref find_dependency_roots(), etc.
     * 4. Optionally export the graph in DOT format using \ref write_dot() for visualization.
     *
     * \note **Return types:** Query methods return \ref element_info_t (a `std::variant<node_info_t, topic_info_t>`)
     * which contains a copy of the element's data.
     * Use std::get<node_info_t>(element_info) or std::get<topic_info_t>(element_info) to access the specific type.
     *
     * \see ErrorPublisher for the publishing side that nodes use to report errors.
     * \see ErroGraphViewer at https://github.com/ctu-mrs/mrs_errorgraph_viewer/blob/ros2/src/errorgraph_viewer.cpp
     * for an example of using the Errorgraph and visualizing it.
     */
    class Errorgraph
    {
    public:
      /**
       * \brief Construct an empty Errorgraph.
       * \param clock  Clock used for timestamping and staleness detection.
       */
      explicit Errorgraph(rclcpp::Clock::SharedPtr clock) : clock_(clock)
      {
      }

      /* error_t //{ */

      /**
       * \brief Represents a single error reported by a node.
       *
       * An error can be a general description, or a dependency indicator
       * (waiting for a specific node or topic).
       */
      struct error_t
      {
        rclcpp::Time stamp;                          ///< Timestamp when the error was reported.
        std::string type;                            ///< Error type string (see ErrorgraphError message constants).
        std::optional<std::string> waited_for_topic; ///< Topic name if this is a "waiting for topic" error.
        std::optional<node_id_t> waited_for_node;    ///< Node ID if this is a "waiting for node" error.

        /// \brief Returns true if this error indicates a dependency (waiting for a topic or node).
        inline bool is_waiting_for() const
        {
          return is_waiting_for_topic() || is_waiting_for_node();
        }

        /// \brief Returns true if this error is waiting for a topic.
        inline bool is_waiting_for_topic() const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC;
        }

        /// \brief Returns true if this error is waiting for a node.
        inline bool is_waiting_for_node() const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE;
        }

        /// \brief Returns true if this error is waiting for the given topic.
        inline bool is_waiting_for(const std::string& topic) const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC && waited_for_topic.has_value() && waited_for_topic.value() == topic;
        }

        /// \brief Returns true if this error is waiting for the given node.
        inline bool is_waiting_for(const node_id_t& node_id) const
        {
          return type == errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE && waited_for_node.has_value() && waited_for_node.value() == node_id;
        }

        /// \brief Returns true if this represents a "no error" state.
        inline bool is_no_error() const
        {
          return type == errorgraph_error_msg_t::TYPE_NO_ERROR;
        }

        /// \brief Construct from a ROS message.
        error_t(const errorgraph_error_msg_t& msg) : type(msg.type)
        {
          if (msg.type == errorgraph_error_msg_t::TYPE_WAITING_FOR_NODE)
            waited_for_node = node_id_t::from_msg(msg.waited_for_node);
          else if (msg.type == errorgraph_error_msg_t::TYPE_WAITING_FOR_TOPIC)
            waited_for_topic = msg.waited_for_topic;
          stamp = msg.stamp;
        }

        /// \brief Convert to a ROS message.
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

      /* node_info_t //{ */

      /**
       * \brief Public view of a node element, returned by query methods.
       *
       * Contains a copy of the relevant data from the internal element representation,
       * without exposing graph-traversal internals.
       */
      struct node_info_t
      {
        node_id_t source_node;       ///< Node ID of this element.
        std::vector<error_t> errors; ///< List of errors reported by this node.
        rclcpp::Time stamp;          ///< Last time this element was updated.
        bool not_reporting;          ///< Whether this node has stopped reporting (stale).

        /// \brief Convert to a ROS message.
        errorgraph_element_msg_t to_msg() const;
      };

      //}

      /* topic_info_t //{ */

      /**
       * \brief Public view of a topic element, returned by query methods.
       *
       * Contains a copy of the relevant data from the internal element representation,
       * without exposing graph-traversal internals.
       */
      struct topic_info_t
      {
        std::string topic_name; ///< Topic name.
        node_id_t source_node;  ///< Expected publisher node for this topic.
        rclcpp::Time stamp;     ///< Last time this element was updated.
        bool not_reporting;     ///< Whether this topic's publisher has stopped reporting.

        /// \brief Convert to a ROS message.
        errorgraph_element_msg_t to_msg() const;
      };

      //}

      /// \brief Type-safe variant representing either a node or topic element info.
      using element_info_t = std::variant<node_info_t, topic_info_t>;

      /* element_t //{ */

      /**
       * \brief Internal representation of a node or topic in the error dependency graph.
       *
       * Each element is either a ROS node (identified by \ref node_id_t) or a topic
       * (identified by name). Node elements carry a list of errors; topic elements
       * serve as intermediate vertices connecting nodes in the dependency graph.
       *
       * \note This type is used internally for graph operations. Public API methods
       * return \ref element_info_t instead.
       */
      struct element_t
      {
        /// \brief Whether this element represents a node or a topic.
        enum class type_t
        {
          node,
          topic
        };

        size_t element_id; ///< Unique ID of this element within the graph.
        type_t type;       ///< Whether this is a node or topic element.

        std::string topic_name; ///< Topic name (only meaningful if type == topic).
        node_id_t source_node;  ///< Source node ID, or expected publisher for a topic.

        std::vector<error_t> errors; ///< List of errors reported by this element (nodes only).
        rclcpp::Time stamp;          ///< Last time this element was updated from a message.
        static constexpr double DEFAULT_NOT_REPORTING_DELAY_SECONDS = 3.0;
        rclcpp::Duration not_reporting_delay = rclcpp::Duration::from_seconds(DEFAULT_NOT_REPORTING_DELAY_SECONDS);

        std::vector<element_t*> parents;  ///< Parent elements in the dependency graph.
        std::vector<element_t*> children; ///< Child elements in the dependency graph.
        bool visited = false;             ///< Visited flag used during graph traversal.

        rclcpp::Clock::SharedPtr clock_;

        /// \brief Construct a node element.
        element_t(size_t element_id, const node_id_t& source_node, rclcpp::Clock::SharedPtr clock)
            : element_id(element_id), type(type_t::node), source_node(source_node), stamp(static_cast<int64_t>(0), clock->get_clock_type()), clock_(clock){};

        /// \brief Construct a topic element.
        element_t(size_t element_id, const std::string& topic_name, const node_id_t& source_node, rclcpp::Clock::SharedPtr clock)
            : element_id(element_id), type(type_t::topic), topic_name(topic_name), source_node(source_node),
              stamp(static_cast<int64_t>(0), clock->get_clock_type()), clock_(clock){};

        /// \brief Returns pointers to topic names this element is waiting for.
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

        /// \brief Returns pointers to node IDs this element is waiting for.
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

        /// \brief Returns true if this element has any dependency errors.
        inline bool is_waiting_for() const
        {
          return std::any_of(std::begin(errors), std::end(errors), [](const auto& error) { return error.is_waiting_for(); });
        }

        /// \brief Returns true if this element is waiting for the given node.
        inline bool is_waiting_for(const node_id_t& node_id) const
        {
          return std::any_of(std::begin(errors), std::end(errors), [node_id](const auto& error) { return error.is_waiting_for(node_id); });
        }

        /// \brief Returns true if this element has no errors and is actively reporting.
        inline bool is_no_error() const
        {
          return !is_not_reporting() && std::all_of(std::begin(errors), std::end(errors), [](const auto& error) { return error.is_no_error(); });
        }

        /// \brief Returns true if this node element has stopped reporting (stale).
        inline bool is_not_reporting() const
        {
          return type != type_t::topic && clock_->now() - stamp > not_reporting_delay;
        }

        /// \brief Convert to a ROS message.
        errorgraph_element_msg_t to_msg() const
        {
          errorgraph_element_msg_t ret;
          ret.stamp = stamp;
          ret.source_node = source_node.to_msg();
          for (const auto& error : errors)
            ret.errors.push_back(error.to_msg());
          return ret;
        }

        /// \brief Convert to a public element_info_t variant.
        element_info_t to_info() const;
      };

      //}

    private:
      std::vector<std::unique_ptr<element_t>> elements_;
      bool graph_up_to_date_ = false;
      rclcpp::Clock::SharedPtr clock_;

      std::vector<element_t*> find_elements_waited_for(const element_t& by_element);

      element_t* find_element_mutable(const std::string& topic_name);

      element_t* find_element_mutable(const node_id_t& node_id);

      void prepare_graph();

      void build_graph();

      std::vector<const element_t*> DFS(element_t* from, bool* loop_detected_out = nullptr);

      element_t* add_new_element(const std::string& topic_name, const node_id_t& node_id = {});

      element_t* add_new_element(const node_id_t& node_id);

      size_t last_element_id = 0;

    public:
      /**
       * \brief Write the graph in Graphviz DOT format.
       * \param os  Output stream to write the DOT representation to.
       */
      void write_dot(std::ostream& os);

      /**
       * \brief Find the root-cause elements blocking the given node.
       *
       * Traverses the dependency graph from the specified node to find leaf elements
       * (elements with errors that don't depend on anything else).
       *
       * \param node_id             The node to trace dependencies for.
       * \param loop_detected_out   If non-null, set to true when a cycle is detected.
       * \return  Copies of root-cause element info as type-safe variants.
       */
      std::vector<element_info_t> find_dependency_roots(const node_id_t& node_id, bool* loop_detected_out = nullptr);

      /**
       * \brief Find all root-cause elements across the entire graph.
       * \return  Copies of elements that have errors and are not blocked by other elements.
       */
      std::vector<element_info_t> find_error_roots();

      /**
       * \brief Find all root elements (elements with no parents in the dependency graph).
       * \return  Copies of root element info as type-safe variants.
       */
      std::vector<element_info_t> find_roots();

      /**
       * \brief Find all leaf elements (elements with no children in the dependency graph).
       * \return  Copies of leaf element info as type-safe variants.
       */
      std::vector<element_info_t> find_leaves();

      /**
       * \brief Find an element by topic name.
       * \param topic_name  The topic name to search for.
       * \return  A copy of the element info, or std::nullopt if not found.
       */
      std::optional<element_info_t> find_element(const std::string& topic_name);

      /**
       * \brief Find an element by node ID.
       * \param node_id  The node ID to search for.
       * \return  A copy of the element info, or std::nullopt if not found.
       */
      std::optional<element_info_t> find_element(const node_id_t& node_id);

      /**
       * \brief Add or update an element from a received ROS message.
       *
       * If an element with the same source node already exists, its errors and timestamp
       * are updated. Otherwise, a new element is created and any dependency edges implied
       * by the errors are added to the graph.
       *
       * \param msg  The received ErrorgraphElement message.
       * \return  A copy of the added or updated element info.
       */
      element_info_t add_element_from_msg(const errorgraph_element_msg_t& msg);
    };

  } // namespace errorgraph
} // namespace mrs_lib
