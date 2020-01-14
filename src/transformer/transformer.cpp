#include <mrs_lib/transformer.h>
#include <mrs_lib/GpsConversions.h>

namespace mrs_lib
{

// | ----------------------- Transformer ---------------------- |

/* Transformer() (constructors)//{ */

Transformer::Transformer() {
}

Transformer::~Transformer() {
}

Transformer::Transformer(const std::string node_name, const std::string uav_name, const double cache_timeout) {

  this->node_name_     = node_name;
  this->uav_name_      = uav_name;
  this->cache_timeout_ = cache_timeout;

  if (uav_name.compare("") != STRING_EQUAL) {
    this->got_uav_name_ = true;
  } else {
    this->got_uav_name_ = false;
  }

  this->current_control_frame_ = "";

  this->tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, node_name);

  this->is_initialized_ = true;
}

Transformer::Transformer(const std::string node_name, const std::string uav_name) : Transformer(node_name, uav_name, 0.001){};

Transformer::Transformer(const std::string node_name, const double cache_timeout) : Transformer(node_name, "", cache_timeout){};

Transformer::Transformer(const std::string node_name) : Transformer(node_name, "", 0.001){};

Transformer::Transformer(const Transformer& other) {

  this->is_initialized_ = other.is_initialized_;
  this->node_name_      = other.node_name_;
  this->uav_name_       = other.uav_name_;
  this->cache_timeout_  = other.cache_timeout_;
  this->got_uav_name_   = other.got_uav_name_;

  {
    std::scoped_lock lock(other.mutex_current_control_frame_);

    this->current_control_frame_     = other.current_control_frame_;
    this->got_current_control_frame_ = other.got_current_control_frame_;
  }

  if (this->is_initialized_) {
    this->tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this->node_name_);
  }
}

Transformer& Transformer::operator=(const Transformer& other) {

  if (this == &other) {
    return *this;
  }

  this->is_initialized_ = other.is_initialized_;
  this->node_name_      = other.node_name_;
  this->uav_name_       = other.uav_name_;
  this->cache_timeout_  = other.cache_timeout_;
  this->got_uav_name_   = other.got_uav_name_;

  {
    std::scoped_lock lock(other.mutex_current_control_frame_);

    this->current_control_frame_     = other.current_control_frame_;
    this->got_current_control_frame_ = other.got_current_control_frame_;
  }

  if (this->is_initialized_) {
    this->tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this->node_name_);
  }

  return *this;
}

//}

/* transformReferenceSingle() //{ */

bool Transformer::transformReferenceSingle(const std::string to_frame, mrs_msgs::ReferenceStamped& ref) {

  if (!is_initialized_) {
    ROS_ERROR("[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return false;
  }

  ref.header.frame_id           = resolveFrameName(ref.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (to_frame_resolved.compare(ref.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  mrs_lib::TransformStamped tf;

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

  if (!is_initialized_) {
    ROS_ERROR("[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return false;
  }

  ref.header.frame_id           = resolveFrameName(ref.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (to_frame_resolved.compare(ref.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  mrs_lib::TransformStamped tf;

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

  if (!is_initialized_) {
    ROS_ERROR("[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return false;
  }

  ref.header.frame_id           = resolveFrameName(ref.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (to_frame_resolved.compare(ref.header.frame_id) == STRING_EQUAL) {
    return true;
  }

  mrs_lib::TransformStamped tf;

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

bool Transformer::transformReference(const mrs_lib::TransformStamped& tf, mrs_msgs::ReferenceStamped& ref) {

  if (!is_initialized_) {
    ROS_ERROR("[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return false;
  }

  ref.header.frame_id = resolveFrameName(ref.header.frame_id);

  if (ref.header.frame_id.compare(tf.to()) == STRING_EQUAL) {
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

    geometry_msgs::TransformStamped transform = tf.getTransform();

    if (transformPoseImpl(tf, pose)) {

      // copy the new transformed data back
      ref.reference.position.x = pose.pose.position.x;
      ref.reference.position.y = pose.pose.position.y;
      ref.reference.position.z = pose.pose.position.z;

      quaternionMsgToTF(pose.pose.orientation, quat);
      tf::Matrix3x3 m(quat);
      double        roll, pitch;
      m.getRPY(roll, pitch, ref.reference.yaw);

      ref.header.frame_id = transform.header.frame_id;
      ref.header.stamp    = transform.header.stamp;

      return true;

    } else {

      return false;
    }
  }
  catch (...) {
    ROS_WARN("[%s]: Transformer: Error during transform from \"%s\" frame to \"%s\" frame.", node_name_.c_str(), tf.from().c_str(), tf.to().c_str());
    return false;
  }
}

//}

/* transformPoseImpl() //{ */

bool Transformer::transformPoseImpl(const mrs_lib::TransformStamped& tf, geometry_msgs::PoseStamped& pose) {

  pose.header.frame_id          = resolveFrameName(pose.header.frame_id);
  std::string latlon_frame_name = resolveFrameName(LATLON_ORIGIN);

  // if it is already in the target frame
  if (pose.header.frame_id.compare(tf.to()) == STRING_EQUAL) {
    return true;
  }

  // check for the transform from LAT-LON GPS
  if (tf.from().compare(latlon_frame_name) == STRING_EQUAL) {

    ROS_INFO("[%s]: transforming from latlon", ros::this_node::getName().c_str());

    double utm_x, utm_y;

    // convert LAT-LON to UTM
    mrs_lib::UTM(pose.pose.position.x, pose.pose.position.y, &utm_x, &utm_y);

    std::string utm_frame_name = resolveFrameName("utm_origin");

    pose.header.frame_id = utm_frame_name;
    pose.pose.position.x = utm_x;
    pose.pose.position.y = utm_y;

    mrs_lib::TransformStamped utm_origin_to_end_tf;
    if (getTransform(utm_frame_name, tf.to(), tf.stamp(), utm_origin_to_end_tf)) {

      return transformPoseImpl(utm_origin_to_end_tf, pose);

    } else {

      return false;
    }

    // it is a normal transform
  } else if (tf.to().compare(latlon_frame_name) == STRING_EQUAL) {

    if (!got_utm_zone_) {

      ROS_ERROR_THROTTLE(1.0, "[%s]: cannot transform to latlong, missing UTM zone (did you call setCurrentLatLon()?)", node_name_.c_str());
      return false;
    }

    std::string utm_frame_name = resolveFrameName("utm_origin");

    mrs_lib::TransformStamped start_to_utm_origin_tf;
    if (getTransform(tf.from(), utm_frame_name, tf.stamp(), start_to_utm_origin_tf)) {

      if (transformPoseImpl(start_to_utm_origin_tf, pose)) {

        std::scoped_lock lock(mutex_utm_zone_);

        double lat, lon;

        mrs_lib::UTMtoLL(pose.pose.position.y, pose.pose.position.x, utm_zone_, lat, lon);

        pose.pose.position.x = lat;
        pose.pose.position.y = lon;

        return true;

      } else {
        return false;
      }

    } else {

      return false;
    }

  } else {

    // create the pose message
    geometry_msgs::PoseStamped pose_transformed = pose;

    try {
      geometry_msgs::TransformStamped transform = tf.getTransform();

      tf2::doTransform(pose_transformed, pose_transformed, transform);

      // copy the new transformed data back
      pose = pose_transformed;

      return true;
    }
    catch (...) {
      ROS_WARN("[%s]: Transformer: Error during transform from \"%s\" frame to \"%s\" frame.", node_name_.c_str(), tf.from().c_str(), tf.to().c_str());
      return false;
    }
  }
}

//}

/* transformPose() //{ */

bool Transformer::transformPose(const mrs_lib::TransformStamped& tf, geometry_msgs::PoseStamped& pose) {

  return transformPoseImpl(tf, pose);
}

//}

/* transformVector3() //{ */

bool Transformer::transformVector3(const mrs_lib::TransformStamped& tf, geometry_msgs::Vector3Stamped& vector3) {

  if (!is_initialized_) {
    ROS_ERROR("[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return false;
  }

  vector3.header.frame_id = resolveFrameName(vector3.header.frame_id);

  if (vector3.header.frame_id.compare(tf.to()) == STRING_EQUAL) {
    return true;
  }

  // create the pose message
  geometry_msgs::Vector3Stamped vector3_transformed = vector3;

  try {
    geometry_msgs::TransformStamped transform = tf.getTransform();

    tf2::doTransform(vector3_transformed, vector3_transformed, transform);

    // copy the new transformed data back
    vector3 = vector3_transformed;

    return true;
  }
  catch (...) {
    ROS_WARN("[%s]: Transformer: Error during transform from \"%s\" frame to \"%s\" frame.", node_name_.c_str(), tf.from().c_str(), tf.to().c_str());
    return false;
  }
}

//}

/* getTransform() //{ */

bool Transformer::getTransform(const std::string from_frame, const std::string to_frame, const ros::Time time_stamp, mrs_lib::TransformStamped& tf) {

  if (!is_initialized_) {
    ROS_ERROR("[%s]: Transformer: cannot provide transform, not initialized", ros::this_node::getName().c_str());
    return false;
  }

  std::string to_frame_resolved     = resolveFrameName(to_frame);
  std::string from_frame_resolved   = resolveFrameName(from_frame);
  std::string latlon_frame_resolved = resolveFrameName(LATLON_ORIGIN);

  // check for latlon transform
  if (from_frame_resolved.compare(latlon_frame_resolved) == STRING_EQUAL || to_frame_resolved.compare(latlon_frame_resolved) == STRING_EQUAL) {

    tf = mrs_lib::TransformStamped(from_frame_resolved, to_frame_resolved, time_stamp);
    return true;
  }

  std::string tf_frame_combined = to_frame_resolved + "->" + from_frame_resolved;

  try {

    std::scoped_lock lock(mutex_tf_buffer_);

    // get the transform
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(to_frame_resolved, from_frame_resolved, time_stamp);

    // return it
    tf = mrs_lib::TransformStamped(from_frame_resolved, to_frame_resolved, ros::Time::now(), transform);

    return true;
  }
  catch (tf2::TransformException& ex) {
    // this happens often -> DEBUG
    ROS_DEBUG("[%s]: Transformer: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(), from_frame_resolved.c_str(),
              to_frame_resolved.c_str(), ex.what());
  }

  try {

    std::scoped_lock lock(mutex_tf_buffer_);

    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(to_frame_resolved, from_frame_resolved, ros::Time(0));

    // return it
    tf = mrs_lib::TransformStamped(from_frame_resolved, to_frame_resolved, ros::Time::now(), transform);

    return true;
  }
  catch (tf2::TransformException& ex) {
    // this does not happen often and when it does, it should be seen
    ROS_ERROR("[%s]: Transformer: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(), from_frame_resolved.c_str(),
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

      ROS_ERROR("[%s]: Transformer: could not resolve an empty frame_id, missing the current control frame (are you calling the setCurrentControlFrame()?)",
                node_name_.c_str());

      return "";
    }
  }

  if (in.substr(0, 3).compare("uav") != STRING_EQUAL) {

    if (got_uav_name_) {

      return uav_name_ + "/" + in;

    } else {

      ROS_ERROR("[%s]: Transformer: could not deduce a namespaced frame_id '%s' (did you instanced the Transformer with the uav_name argument?)",
                node_name_.c_str(), in.c_str());
    }
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

/* setCurrentLatLon() //{ */

void Transformer::setCurrentLatLon(const double lat, const double lon) {

  double utm_x, utm_y;

  {
    std::scoped_lock lock(mutex_utm_zone_);

    LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_);

    got_utm_zone_ = true;
  }
}

//}

// | --------------- TransformStamped wrapper ---------------- |

/* TransformStamped() //{ */

TransformStamped::TransformStamped() {
}

TransformStamped::TransformStamped(const std::string from_frame, const std::string to_frame, ros::Time stamp) {

  this->from_frame_ = from_frame;
  this->to_frame_   = to_frame;
  this->stamp_      = stamp;
}

TransformStamped::TransformStamped(const std::string from_frame, const std::string to_frame, ros::Time stamp,
                                   const geometry_msgs::TransformStamped transform_stamped) {

  this->from_frame_        = from_frame;
  this->to_frame_          = to_frame;
  this->transform_stamped_ = transform_stamped;
  this->stamp_             = stamp;
}

//}

/* from() //{ */

std::string TransformStamped::from(void) const {

  return from_frame_;
}

//}

/* to() //{ */

std::string TransformStamped::to(void) const {

  return to_frame_;
}

//}

/* stamp() //{ */

ros::Time TransformStamped::stamp(void) const {

  return stamp_;
}

//}

/* getTransform() //{ */

geometry_msgs::TransformStamped TransformStamped::getTransform(void) const {

  return transform_stamped_;
}

//}
}  // namespace mrs_lib
