#include <mrs_lib/transformer.h>

namespace mrs_lib
{

/* Transformer() //{ */

Transformer::Transformer(const std::string node_name, const std::string uav_name, const double cache_timeout) {

  this->uav_name_  = uav_name;
  this->node_name_ = node_name;

  this->current_control_frame_ = "";

  this->cache_timeout_ = cache_timeout;

  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, node_name);

  ROS_INFO("[%s]: tf transformer initialized", node_name_.c_str());
}

//}

/* ~Transformer() //{ */

Transformer::~Transformer() {
}

//}

/* transformReferenceSingle() //{ */

bool Transformer::transformReferenceSingle(const std::string to_frame, mrs_msgs::ReferenceStamped& ref) {

  ref.header.frame_id           = resolveFrameName(ref.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (to_frame_resolved.compare(ref.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  geometry_msgs::TransformStamped tf;

  // get the transform
  if (!getTransform(ref.header.frame_id, to_frame_resolved, ref.header.stamp, tf)) {
    return false;
  }

  // do the transformation
  if (!transformReference(tf, ref)) {
    return false;
  }

  return true;
}

//}

/* transformPoseSingle() //{ */

bool Transformer::transformPoseSingle(const std::string to_frame, geometry_msgs::PoseStamped& ref) {

  ref.header.frame_id           = resolveFrameName(ref.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (to_frame_resolved.compare(ref.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  geometry_msgs::TransformStamped tf;

  // get the transform
  if (!getTransform(ref.header.frame_id, to_frame_resolved, ref.header.stamp, tf)) {
    return false;
  }

  // do the transformation
  if (!transformPose(tf, ref)) {
    return false;
  }

  return true;
}

//}

/* transformVector3Single() //{ */

bool Transformer::transformVector3Single(const std::string to_frame, geometry_msgs::Vector3Stamped& ref) {

  ref.header.frame_id           = resolveFrameName(ref.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (to_frame_resolved.compare(ref.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  geometry_msgs::TransformStamped tf;

  // get the transform
  if (!getTransform(ref.header.frame_id, to_frame_resolved, ref.header.stamp, tf)) {
    return false;
  }

  // do the transformation
  if (!transformVector3(tf, ref)) {
    return false;
  }

  return true;
}

//}

/* transformReference() //{ */

bool Transformer::transformReference(const geometry_msgs::TransformStamped& tf, mrs_msgs::ReferenceStamped& ref) {

  ref.header.frame_id = resolveFrameName(ref.header.frame_id);

  if (tf.header.frame_id.compare(ref.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  // create the pose message
  geometry_msgs::PoseStamped pose;
  pose.header = ref.header;

  pose.pose.position.x = ref.reference.position.x;
  pose.pose.position.y = ref.reference.position.y;
  pose.pose.position.z = ref.reference.position.z;

  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, ref.reference.yaw);

  pose.pose.orientation.x = quat.getX();
  pose.pose.orientation.y = quat.getY();
  pose.pose.orientation.z = quat.getZ();
  pose.pose.orientation.w = quat.getW();

  try {
    tf2::doTransform(pose, pose, tf);

    // copy the new transformed data back
    ref.reference.position.x = pose.pose.position.x;
    ref.reference.position.y = pose.pose.position.y;
    ref.reference.position.z = pose.pose.position.z;

    quaternionMsgToTF(pose.pose.orientation, quat);
    tf::Matrix3x3 m(quat);
    double        roll, pitch;
    m.getRPY(roll, pitch, ref.reference.yaw);

    ref.header.frame_id = tf.header.frame_id;
    ref.header.stamp    = tf.header.stamp;

    return true;
  }
  catch (...) {
    ROS_WARN("[%s]: Error during transform from \"%s\" frame to \"%s\" frame.", node_name_.c_str(), tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    return false;
  }
}

//}

/* transformPose() //{ */

bool Transformer::transformPose(const geometry_msgs::TransformStamped& tf, geometry_msgs::PoseStamped& pose) {

  pose.header.frame_id = resolveFrameName(pose.header.frame_id);

  if (tf.header.frame_id.compare(pose.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  // create the pose message
  geometry_msgs::PoseStamped pose_transformed = pose;

  try {
    tf2::doTransform(pose_transformed, pose_transformed, tf);

    // copy the new transformed data back
    pose = pose_transformed;

    return true;
  }
  catch (...) {
    ROS_WARN("[%s]: Error during transform from \"%s\" frame to \"%s\" frame.", node_name_.c_str(), tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    return false;
  }
}

//}

/* transformVector3() //{ */

bool Transformer::transformVector3(const geometry_msgs::TransformStamped& tf, geometry_msgs::Vector3Stamped& vector3) {

  vector3.header.frame_id = resolveFrameName(vector3.header.frame_id);

  if (tf.header.frame_id.compare(vector3.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  // create the pose message
  geometry_msgs::Vector3Stamped vector3_transformed = vector3;

  try {
    tf2::doTransform(vector3_transformed, vector3_transformed, tf);

    // copy the new transformed data back
    vector3 = vector3_transformed;

    return true;
  }
  catch (...) {
    ROS_WARN("[%s]: Error during transform from \"%s\" frame to \"%s\" frame.", node_name_.c_str(), tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
    return false;
  }
}

//}

/* getTransform() //{ */

bool Transformer::getTransform(const std::string from_frame, const std::string to_frame, const ros::Time time_stamp, geometry_msgs::TransformStamped& tf) {

  std::string to_frame_resolved   = resolveFrameName(to_frame);
  std::string from_frame_resolved = resolveFrameName(from_frame);

  std::string tf_frame_combined = to_frame_resolved + "->" + from_frame_resolved;

  // check the cache
  std::map<std::string, TransformCache_t>::iterator it;

  {
    std::scoped_lock lock(mutex_transformer_cache_);

    it = transformer_cache_.find(tf_frame_combined);

    if (it != transformer_cache_.end()) {  // found in the cache

      double tf_age = (ros::Time::now() - it->second.stamp).toSec();

      if (tf_age < cache_timeout_) {

        tf = it->second.tf;
        return true;
      }

    } else {

      TransformCache_t new_tf;
      new_tf.stamp = ros::Time(0);
      transformer_cache_.insert(std::pair<std::string, TransformCache_t>(tf_frame_combined, new_tf));

      it = transformer_cache_.find(tf_frame_combined);
    }
  }

  try {

    std::scoped_lock lock(mutex_tf_buffer_);

    tf = tf_buffer_.lookupTransform(to_frame_resolved, from_frame_resolved, time_stamp, ros::Duration(0.0));

    it->second.stamp = ros::Time::now();
    it->second.tf    = tf;

    return true;
  }
  catch (tf2::TransformException& ex) {
    // this happens often -> DEBUG
    ROS_DEBUG("[%s]: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(), from_frame_resolved.c_str(),
              to_frame_resolved.c_str(), ex.what());
  }

  try {

    std::scoped_lock lock(mutex_tf_buffer_);

    tf = tf_buffer_.lookupTransform(to_frame_resolved, from_frame_resolved, ros::Time(0), ros::Duration(0.0));

    it->second.stamp = ros::Time::now();
    it->second.tf    = tf;

    return true;
  }
  catch (tf2::TransformException& ex) {
    // this does not happen often and when it does, it should be seen
    ROS_ERROR("[%s]: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(), from_frame_resolved.c_str(),
              to_frame_resolved.c_str(), ex.what());
  }

  return false;
}

//}

/* resolveFrameName() //{ */

std::string Transformer::resolveFrameName(const std::string in) {

  if (in.compare("") == STRING_EQUAL) {

    if (got_current_control_frame_) {

      std::scoped_lock lock(mutex_current_control_frame_);

      return current_control_frame_;

    } else {

      ROS_ERROR("[%s]: could not resolve an empty frame_id, missing the current control frame (are you calling the setCurrentControlFrame()?)",
                node_name_.c_str());

      return "";
    }
  }

  size_t found = in.find("/");
  if (found == std::string::npos) {

    return uav_name_ + "/" + in;
  }

  return in;
}

//}

/* setCurrentControlFrame() //{ */

void Transformer::setCurrentControlFrame(const std::string in) {

  std::scoped_lock lock(mutex_current_control_frame_);

  current_control_frame_ = in;

  got_current_control_frame_ = true;
}

//}

}  // namespace mrs_lib
