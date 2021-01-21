#include <mrs_lib/transform_broadcaster.h>

namespace mrs_lib
{

/* constructor //{ */
TransformBroadcaster::TransformBroadcaster() {
  broadcaster_ = tf2_ros::TransformBroadcaster();
}
//}

/* sendTransform (const geometry_msgs::TransformStamped &transform) //{ */
void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped &transform) {
  std::string frames_from_to = transform.header.frame_id + transform.child_frame_id;
  if (last_messages_.count(frames_from_to) > 0) {
    if (transform.header.stamp > last_messages_[frames_from_to]) {
      broadcaster_.sendTransform(transform);
      last_messages_[frames_from_to] = transform.header.stamp;
    } else {
      ROS_WARN_ONCE("[%s]: TF_REPEATED_DATA ignoring data with redundant timestamp. Transform from frame '%s' to frame '%s'", ros::this_node::getName().c_str(),
                    transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    }
  } else {
    std::pair<std::string, ros::Time> new_pair;
    new_pair.first  = frames_from_to;
    new_pair.second = transform.header.stamp;
    broadcaster_.sendTransform(transform);
    last_messages_.insert(new_pair);
  }
}
//}

/* sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms) //{ */
void TransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms) {
  for (auto transform : transforms) {
    std::string frames_from_to = transform.header.frame_id + transform.child_frame_id;
    if (last_messages_.count(frames_from_to) > 0) {
      if (transform.header.stamp > last_messages_[frames_from_to]) {
        broadcaster_.sendTransform(transform);
        last_messages_[frames_from_to] = transform.header.stamp;
      } else {
        ROS_WARN_ONCE("[%s]: TF_REPEATED_DATA ignoring data with redundant timestamp. Transform from frame '%s' to frame '%s'",
                      ros::this_node::getName().c_str(), transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
      }
    } else {
      std::pair<std::string, ros::Time> new_pair;
      new_pair.first  = frames_from_to;
      new_pair.second = transform.header.stamp;
      broadcaster_.sendTransform(transform);
      last_messages_.insert(new_pair);
    }
  }
}
//}

}  // namespace mrs_lib
