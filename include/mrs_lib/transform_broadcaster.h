#ifndef TRANSFORM_BROADCASTER_H
#define TRANSFORM_BROADCASTER_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace mrs_lib
{

/* TransformBroadcaster //{ */

/**
 * @brief Wrapper for the tf2_ros::TransformBroadcaster.
 * With each sendTransform() command, the message is checked against the last message with the same frame IDs.
 * If the transform was already published in this ros::Time step, then the transform is skipped.
 * Prevents endless stream of warnings from spamming the console output.
 */
class TransformBroadcaster {

public:
  //! is thrown when calculating of heading is not possible due to atan2 exception
  struct NotInitializedException : public std::exception
  {
    const char *what() const throw() {
      return "TransformBroadcaster: not initialized! Call the constructor and pass the node pointer.";
    }
  };

  /**
   * @brief constructor, internally starts the TransformBroadcaster
   */
  TransformBroadcaster();

  /**
   * @brief constructor, internally starts the TransformBroadcaster
   */
  TransformBroadcaster(const rclcpp::Node::SharedPtr &node);

  /**
   * @brief check if the transform is newer than the last published one and publish it. Transform is skipped if a duplicit timestamp is found
   *
   * @param transform to be published
   */
  void sendTransform(const geometry_msgs::msg::TransformStamped &transform);

  /**
   * @brief check if the transforms are newer than the last published ones and publish them. A transform is skipped if a duplicit timestamp is found
   *
   * @param transforms vector of transforms to be published
   */
  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> &transforms);

private:
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief Internaly, the tf2_ros TransformBroadcaster is still used
   */
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  /**
   * @brief Map of transformations and their last used timestamp.
   * Dynamically adjusts with new frames.
   * Key - string: a combined frame_id (transform from) and child_frame_id (tranform to)
   * Value - ros::Time: last timestamp used for this frame pair
   */
  std::map<std::string, rclcpp::Time> last_messages_;
};
//}

}  // namespace mrs_lib

#endif  // TRANSFORM_BROADCASTER_H
