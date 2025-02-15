#include <mrs_lib/transform_broadcaster.h>

namespace mrs_lib
{

/* constructor //{ */

TransformBroadcaster::TransformBroadcaster() {
}

TransformBroadcaster::TransformBroadcaster(const rclcpp::Node::SharedPtr &node) {

  node_        = node;
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

//}

/* sendTransform (const geometry_msgs::TransformStamped &transform) //{ */

void TransformBroadcaster::sendTransform(const geometry_msgs::msg::TransformStamped &transform) {

  if (node_ == nullptr) {
    throw NotInitializedException();
  }

  std::string frames_from_to = transform.header.frame_id + transform.child_frame_id;

  if (last_messages_.count(frames_from_to) > 0) {

    if (rclcpp::Time(transform.header.stamp) > last_messages_[frames_from_to]) {

      broadcaster_->sendTransform(transform);
      last_messages_[frames_from_to] = transform.header.stamp;

    } else {

      RCLCPP_WARN_ONCE(node_->get_logger(), "TF_REPEATED_DATA ignoring data with redundant timestamp. Transform from frame '%s' to frame '%s'",
                       transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    }

  } else {

    std::pair<std::string, rclcpp::Time> new_pair;
    new_pair.first  = frames_from_to;
    new_pair.second = transform.header.stamp;
    broadcaster_->sendTransform(transform);
    last_messages_.insert(new_pair);
  }
}

//}

/* sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms) //{ */

void TransformBroadcaster::sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> &transforms) {

  if (node_ == nullptr) {
    throw NotInitializedException();
  }

  for (auto transform : transforms) {

    std::string frames_from_to = transform.header.frame_id + transform.child_frame_id;

    if (last_messages_.count(frames_from_to) > 0) {

      if (rclcpp::Time(transform.header.stamp) > last_messages_[frames_from_to]) {

        broadcaster_->sendTransform(transform);
        last_messages_[frames_from_to] = transform.header.stamp;

      } else {
        RCLCPP_WARN_ONCE(node_->get_logger(), "TF_REPEATED_DATA ignoring data with redundant timestamp. Transform from frame '%s' to frame '%s'",
                         transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
      }

    } else {

      std::pair<std::string, rclcpp::Time> new_pair;
      new_pair.first  = frames_from_to;
      new_pair.second = transform.header.stamp;
      broadcaster_->sendTransform(transform);
      last_messages_.insert(new_pair);
    }
  }
}

//}

}  // namespace mrs_lib
