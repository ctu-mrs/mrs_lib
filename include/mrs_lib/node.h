#ifndef NODE_H
#define NODE_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace mrs_lib
{

class Node
{
protected:
  Node(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : node_(std::make_shared<rclcpp::Node>(node_name, options))
  {
  }

  Node(
    const std::string& node_name, const std::string& namespace_,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : node_(std::make_shared<rclcpp::Node>(node_name, namespace_, options))
  {
  }

  ~Node()                      = default;
  Node(const Node&)            = default;
  Node& operator=(const Node&) = default;
  Node(Node&&)                 = default;
  Node& operator=(Node&&)      = default;

  rclcpp::Node& this_node() { return *node_; }
  rclcpp::Node::SharedPtr this_node_ptr() const { return node_; }

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace mrs_lib

#endif
