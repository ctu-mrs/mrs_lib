#pragma once
#include <ostream>
#include <mrs_msgs/msg/errorgraph_node_id.hpp>

namespace mrs_lib
{
  namespace errorgraph
  {
    using errorgraph_node_id_msg_t = mrs_msgs::msg::ErrorgraphNodeID;

    /**
     * \brief Identifies a specific component within a ROS node for error reporting.
     *
     * A single ROS node (e.g. "EstimationManager") may contain multiple logical components
     * (e.g. "lat_gps", "alt_garmin") that report errors independently. The node_id_t
     * uniquely identifies such a component within the Errorgraph.
     *
     * Formatted as "node.component" for display (e.g. "EstimationManager.lat_gps").
     */
    struct node_id_t
    {
      std::string node;       ///< Name of the ROS node (e.g. "EstimationManager", "HwApiManager").
      std::string component;  ///< Name of the logical component within the node (e.g. "lat_gps", "main").

      /// \brief Equality comparison by both node and component.
      inline bool operator==(const node_id_t& other) const
      {
        return node == other.node && component == other.component;
      }

      /// \brief Construct from a ROS message.
      static inline node_id_t from_msg(const errorgraph_node_id_msg_t& msg)
      {
        node_id_t ret;
        ret.node = msg.node;
        ret.component = msg.component;
        return ret;
      }

      /// \brief Convert to a ROS message.
      inline errorgraph_node_id_msg_t to_msg() const
      {
        errorgraph_node_id_msg_t ret;
        ret.node = node;
        ret.component = component;
        return ret;
      }
    };

    /// \brief Stream output operator, formats as "node.component".
    inline std::ostream& operator<<(std::ostream& os, const node_id_t& node_id)
    {
      return os << node_id.node << "." << node_id.component;
    }
  } // namespace errorgraph
} // namespace mrs_lib
