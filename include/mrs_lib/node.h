#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mrs_lib
{

  /**
   * \brief Wrapper class around rclcpp::Node providing interface required for components.
   *
   * When deriving directly from rclcpp::Node, the derived class is unable to obtain
   * shared pointer to the rclcpp::Node, because the construction did not yet finish.
   * Since the constructor might need this pointer to initialize subobjects, it cannot
   * derive from rclcpp::Node directly.
   *
   * This class wraps the rclcpp::Node in a way that allows obtaining the shared pointer
   * directly in the constructor. It also provides the interface necessary for the derived
   * class to be used as rclcpp component.
   */
  class Node
  {

  protected:
    /**
     * \brief Constructs the underlying node with the given parameters.
     *
     * See rclcpp::Node for the meaning of individual parameter.
     */
    explicit Node(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : node_(std::make_shared<rclcpp::Node>(node_name, options))
    {
    }

    /**
     * \brief Constructs the underlying node with the given parameters.
     *
     * See rclcpp::Node for the meaning of individual parameter.
     */
    explicit Node(const std::string& node_name, const std::string& namespace_, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : node_(std::make_shared<rclcpp::Node>(node_name, namespace_, options))
    {
    }

    ~Node() = default;
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;
    Node(Node&&) = delete;
    Node& operator=(Node&&) = delete;

    /**
     * \brief Get reference to the underlying rclcpp::Node.
     */
    [[nodiscard]] rclcpp::Node& this_node() noexcept
    {
      return *node_;
    }
    /**
     * \brief Get reference to the underlying rclcpp::Node.
     */
    [[nodiscard]] const rclcpp::Node& this_node() const noexcept
    {
      return *node_;
    }
    /**
     * \brief Get shared pointer to the underlying rclcpp::Node.
     */
    [[nodiscard]] rclcpp::Node::SharedPtr this_node_ptr() const noexcept
    {
      return node_;
    }

    /**
     * \brief Get all timers of the underlying rclcpp::Node. Returns the rclcpp timers from all callback groups of the node (also the ones used by
     * mrs_lib::ROSTimer and by mrs_lib::Timer). The thread-based timers are not included, since they are not registered with the node.
     *
     * \warning The returned pointers keep the timers alive. Use them right away (e.g. to cancel all the timers) and do not store them, otherwise a timer keeps
     * running even after its owner has released it.
     */
    [[nodiscard]] std::vector<rclcpp::TimerBase::SharedPtr> get_timers() const
    {
      std::vector<rclcpp::TimerBase::SharedPtr> timers;

      node_->get_node_base_interface()->for_each_callback_group([&timers](const rclcpp::CallbackGroup::SharedPtr& group) {
        group->find_timer_ptrs_if([&timers](const rclcpp::TimerBase::SharedPtr& timer) {
          timers.push_back(timer);
          return false; // Prevent find_timer_ptrs_if() from stopping after the first timer is found
        });
      });

      return timers;
    }

  public:
    /**
     * This method is required for the class to be treated as rclcpp component.
     *
     * See rclcpp::Node::get_node_base_interface().
     */
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
      return node_->get_node_base_interface();
    }

  private:
    rclcpp::Node::SharedPtr node_;
  };

} // namespace mrs_lib
