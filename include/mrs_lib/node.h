#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace mrs_lib
{

class Node {

protected:
  explicit Node(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : node_(std::make_shared<rclcpp::Node>(node_name, options)) {
  }

  explicit Node(const std::string& node_name, const std::string& namespace_, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : node_(std::make_shared<rclcpp::Node>(node_name, namespace_, options)) {
  }

  ~Node()                      = default;
  Node(const Node&)            = delete;
  Node& operator=(const Node&) = delete;
  Node(Node&&)                 = delete;
  Node& operator=(Node&&)      = delete;

  [[nodiscard]] rclcpp::Node& this_node() noexcept {
    return *node_;
  }
  [[nodiscard]] const rclcpp::Node& this_node() const noexcept {
    return *node_;
  }
  [[nodiscard]] rclcpp::Node::SharedPtr this_node_ptr() const noexcept {
    return node_;
  }

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace mrs_lib
