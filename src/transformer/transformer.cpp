#include <mrs_lib/transformer.h>
#include <mrs_lib/gps_conversions.h>

#include <opencv2/core/types.hpp>

#include <mrs_lib/geometry/conversions.h>
#include <mutex>

using test_t = mrs_msgs::msg::ReferenceStamped;
/* using test_t = pcl::PointCloud<pcl::PointXYZ>; */
template std::optional<test_t>            mrs_lib::Transformer::transform<test_t>(const test_t& what, const geometry_msgs::msg::TransformStamped& to_frame);
template std::optional<test_t::SharedPtr> mrs_lib::Transformer::transform<test_t>(const test_t::SharedPtr&                    what,
                                                                                  const geometry_msgs::msg::TransformStamped& to_frame);
template std::optional<test_t::SharedPtr> mrs_lib::Transformer::transform<test_t>(const test_t::ConstSharedPtr&               what,
                                                                                  const geometry_msgs::msg::TransformStamped& to_frame);
template std::optional<test_t>            mrs_lib::Transformer::transformSingle<test_t>(const test_t& what, const std::string& to_frame);
template std::optional<test_t::SharedPtr> mrs_lib::Transformer::transformSingle<test_t>(const test_t::SharedPtr& what, const std::string& to_frame);
template std::optional<test_t::SharedPtr> mrs_lib::Transformer::transformSingle<test_t>(const test_t::ConstSharedPtr& what, const std::string& to_frame);

#ifdef OPENCV_SPECIALIZATION
namespace tf2
{
template <>
void doTransform(const cv::Point3d& point_in, cv::Point3d& point_out, const geometry_msgs::msg::TransformStamped& transform) {
  const geometry_msgs::msg::Point pt = mrs_lib::geometry::fromCV(point_in);
  geometry_msgs::msg::Point       pt_tfd;
  tf2::doTransform(pt, pt_tfd, transform);
  point_out = mrs_lib::geometry::toCV(pt_tfd);
}
}  // namespace tf2
#endif

namespace mrs_lib
{

// | ----------------------- Transformer ---------------------- |

// | ------------------ user-callable methods ----------------- |

/* Transformer() (constructors)//{ */

Transformer::Transformer() {
}

/* Transformer::Transformer(const rclcpp::Duration& cache_time) */
/*     : initialized_(true), */
/*       tf_buffer_(std::make_unique<tf2_ros::Buffer>(cache_time)), */
/*       tf_listener_ptr_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_)) { */
/* } */

Transformer::Transformer(const rclcpp::Node::SharedPtr& node) : initialized_(true) {

  node_            = node;
  tf_buffer_       = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node, true);
}

Transformer::Transformer(const rclcpp::Node::SharedPtr& node, const rclcpp::Clock::SharedPtr& clock, const rclcpp::Duration& cache_time, const rclcpp::QoS& qos)
    : initialized_(true) {

  node_            = node;
  tf_buffer_       = std::make_unique<tf2_ros::Buffer>(clock, tf2::Duration(cache_time.nanoseconds()), node, qos);
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node, true);
}

Transformer& Transformer::operator=(Transformer&& other) {
  std::scoped_lock lck(other.mutex_, mutex_);

  node_            = std::move(other.node_);
  initialized_     = std::move(other.initialized_);
  tf_buffer_       = std::move(other.tf_buffer_);
  tf_listener_ptr_ = std::move(other.tf_listener_ptr_);

  default_frame_id_    = std::move(other.default_frame_id_);
  prefix_              = std::move(other.prefix_);
  quiet_               = std::move(other.quiet_);
  lookup_timeout_      = std::move(other.lookup_timeout_);
  retry_lookup_newest_ = std::move(other.retry_lookup_newest_);

  got_utm_zone_ = std::move(other.got_utm_zone_);
  utm_zone_     = std::move(other.utm_zone_);

  return *this;
}

//}

/* getTransform() //{ */

std::optional<geometry_msgs::msg::TransformStamped> Transformer::getTransform(const std::string& from_frame_raw, const std::string& to_frame_raw,
                                                                              const rclcpp::Time& time_stamp) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    std::cout << "Transformer: cannot transform, not initialized" << std::endl;
    return std::nullopt;
  }

  // resolve the frames
  const std::string from_frame   = resolveFrameImpl(from_frame_raw);
  const std::string to_frame     = resolveFrameImpl(to_frame_raw);
  const std::string latlon_frame = resolveFrameImpl(LATLON_ORIGIN);

  return getTransformImpl(from_frame, to_frame, time_stamp, latlon_frame);
}

std::optional<geometry_msgs::msg::TransformStamped> Transformer::getTransform(const std::string& from_frame_raw, const rclcpp::Time& from_stamp,
                                                                              const std::string& to_frame_raw, const rclcpp::Time& to_stamp,
                                                                              const std::string& fixed_frame_raw) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    std::cout << "Transformer: cannot transform, not initialized" << std::endl;
    return std::nullopt;
  }

  const std::string from_frame   = resolveFrameImpl(from_frame_raw);
  const std::string to_frame     = resolveFrameImpl(to_frame_raw);
  const std::string fixed_frame  = resolveFrameImpl(fixed_frame_raw);
  const std::string latlon_frame = resolveFrameImpl(LATLON_ORIGIN);

  return getTransformImpl(from_frame, from_stamp, to_frame, to_stamp, fixed_frame, latlon_frame);
}
//}

/* setLatLon() //{ */

void Transformer::setLatLon(const double lat, const double lon) {
  std::scoped_lock lck(mutex_);

  double utm_x, utm_y;
  mrs_lib::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_.data());
  got_utm_zone_ = true;
}

//}

/* transformAsVector() //{ */

[[nodiscard]] std::optional<Eigen::Vector3d> Transformer::transformAsVector(const Eigen::Vector3d& what, const geometry_msgs::msg::TransformStamped& tf) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    std::cout << "Transformer: cannot transform, not initialized" << std::endl;
    return std::nullopt;
  }

  const std::string                          from_frame  = resolveFrameImpl(frame_from(tf));
  const std::string                          to_frame    = resolveFrameImpl(frame_to(tf));
  const geometry_msgs::msg::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);

  const geometry_msgs::msg::Vector3 vec     = mrs_lib::geometry::fromEigenVec(what);
  const auto                        tfd_vec = transformImpl(tf_resolved, vec);
  if (tfd_vec.has_value())
    return mrs_lib::geometry::toEigen(tfd_vec.value());
  else
    return std::nullopt;
}

[[nodiscard]] std::optional<Eigen::Vector3d> Transformer::transformAsVector(const std::string& from_frame_raw, const Eigen::Vector3d& what,
                                                                            const std::string& to_frame_raw, const rclcpp::Time& time_stamp) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    std::cout << "Transformer: cannot transform, not initialized" << std::endl;
    return std::nullopt;
  }

  const std::string from_frame   = resolveFrameImpl(from_frame_raw);
  const std::string to_frame     = resolveFrameImpl(to_frame_raw);
  const std::string latlon_frame = resolveFrameImpl(LATLON_ORIGIN);

  // get the transform
  const auto tf_opt = getTransformImpl(from_frame, to_frame, time_stamp, latlon_frame);
  if (!tf_opt.has_value())
    return std::nullopt;
  const geometry_msgs::msg::TransformStamped& tf = tf_opt.value();

  // do the transformation
  const geometry_msgs::msg::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);
  const geometry_msgs::msg::Vector3          vec         = mrs_lib::geometry::fromEigenVec(what);
  const auto                                 tfd_vec     = transformImpl(tf_resolved, vec);
  if (tfd_vec.has_value())
    return mrs_lib::geometry::toEigen(tfd_vec.value());
  else
    return std::nullopt;
}

/* //} */

/* transformAsPoint() //{ */

[[nodiscard]] std::optional<Eigen::Vector3d> Transformer::transformAsPoint(const Eigen::Vector3d& what, const geometry_msgs::msg::TransformStamped& tf) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    std::cout << "Transformer: cannot transform, not initialized" << std::endl;
    return std::nullopt;
  }

  const std::string                          from_frame  = resolveFrameImpl(frame_from(tf));
  const std::string                          to_frame    = resolveFrameImpl(frame_to(tf));
  const geometry_msgs::msg::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);

  geometry_msgs::msg::Point pt;
  pt.x              = what.x();
  pt.y              = what.y();
  pt.z              = what.z();
  const auto tfd_pt = transformImpl(tf_resolved, pt);
  if (tfd_pt.has_value())
    return mrs_lib::geometry::toEigen(tfd_pt.value());
  else
    return std::nullopt;
}

[[nodiscard]] std::optional<Eigen::Vector3d> Transformer::transformAsPoint(const std::string& from_frame_raw, const Eigen::Vector3d& what,
                                                                           const std::string& to_frame_raw, const rclcpp::Time& time_stamp) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    std::cout << "Transformer: cannot transform, not initialized" << std::endl;
    return std::nullopt;
  }

  const std::string from_frame   = resolveFrameImpl(from_frame_raw);
  const std::string to_frame     = resolveFrameImpl(to_frame_raw);
  const std::string latlon_frame = resolveFrameImpl(LATLON_ORIGIN);

  // get the transform
  const auto tf_opt = getTransformImpl(from_frame, to_frame, time_stamp, latlon_frame);
  if (!tf_opt.has_value())
    return std::nullopt;
  const geometry_msgs::msg::TransformStamped& tf = tf_opt.value();

  // do the transformation
  const geometry_msgs::msg::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);

  geometry_msgs::msg::Point pt;

  pt.x = what.x();
  pt.y = what.y();
  pt.z = what.z();

  const auto tfd_pt = transformImpl(tf_resolved, pt);

  if (tfd_pt.has_value())
    return mrs_lib::geometry::toEigen(tfd_pt.value());
  else
    return std::nullopt;
}

/* //} */

// | ------------- helper implementation methods -------------- |

/* transformImpl() //{ */

/* specialization for mrs_msgs::msg::ReferenceStamped //{ */

std::optional<mrs_msgs::msg::ReferenceStamped> Transformer::transformImpl(const geometry_msgs::msg::TransformStamped& tf,
                                                                          const mrs_msgs::msg::ReferenceStamped&      what) {
  // create a pose message
  geometry_msgs::msg::PoseStamped pose;
  pose.header = what.header;

  pose.pose.position.x  = what.reference.position.x;
  pose.pose.position.y  = what.reference.position.y;
  pose.pose.position.z  = what.reference.position.z;
  pose.pose.orientation = geometry::fromEigen(geometry::quaternionFromHeading(what.reference.heading));

  // try to transform the pose message
  const auto pose_opt = transformImpl(tf, pose);
  if (!pose_opt.has_value())
    return std::nullopt;
  // overwrite the pose with it's transformed value
  pose = pose_opt.value();

  mrs_msgs::msg::ReferenceStamped ret;
  ret.header = pose.header;

  // copy the new transformed data back
  ret.reference.position.x = pose.pose.position.x;
  ret.reference.position.y = pose.pose.position.y;
  ret.reference.position.z = pose.pose.position.z;
  ret.reference.heading    = geometry::headingFromRot(geometry::toEigen(pose.pose.orientation));
  return ret;
}

//}

/* specialization for Eigen::Vector3d //{ */

std::optional<Eigen::Vector3d> Transformer::transformImpl(const geometry_msgs::msg::TransformStamped& tf, const Eigen::Vector3d& what) {
  // just transform it as you would a geometry_msgs::msg::Vector3
  const geometry_msgs::msg::Vector3 as_vec = mrs_lib::geometry::fromEigenVec(what);
  const auto                        opt    = transformImpl(tf, as_vec);
  if (opt.has_value())
    return geometry::toEigen(opt.value());
  else
    return std::nullopt;
}

//}

//}

/* getTransformImpl() //{ */

std::optional<geometry_msgs::msg::TransformStamped> Transformer::getTransformImpl(const std::string& from_frame, const std::string& to_frame,
                                                                                  const rclcpp::Time& time_stamp, const std::string& latlon_frame) {
  if (!initialized_) {
    std::cout << "Transformer: cannot transform, not initialized" << std::endl;
    return std::nullopt;
  }

  // if any of the frames is empty, then the query is invalid, return nullopt
  if (from_frame.empty() || to_frame.empty()) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Transformer: cannot transform, not from_frame or to_frame are empty");
    return std::nullopt;
  }

  // if the frames are the same, just return an identity transform
  if (from_frame == to_frame)
    return create_transform(from_frame, to_frame, time_stamp, tf2::toMsg(tf2::Transform::getIdentity()));

  // check for a transform from/to latlon coordinates - that is a special case handled separately
  if (from_frame == latlon_frame) {
    // find the transformation between the UTM frame and the non-latlon frame to fill the returned tf
    const std::string utm_frame = getFramePrefix(from_frame) + "utm_origin";
    auto              tf_opt    = getTransformImpl(utm_frame, to_frame, time_stamp, latlon_frame);
    if (!tf_opt.has_value())
      return std::nullopt;
    // change the transformation frames to point from latlon
    frame_from(*tf_opt) = from_frame;
    return tf_opt;
  } else if (to_frame == latlon_frame) {
    // find the transformation between the UTM frame and the non-latlon frame to fill the returned tf
    const std::string utm_frame = getFramePrefix(to_frame) + "utm_origin";
    auto              tf_opt    = getTransformImpl(from_frame, utm_frame, time_stamp, latlon_frame);
    if (!tf_opt.has_value())
      return std::nullopt;
    // change the transformation frames to point to latlon
    frame_to(*tf_opt) = to_frame;
    return tf_opt;
  }

  tf2::TransformException ex("");
  // first try to get transform at the requested time
  try {
    // try looking up and returning the transform
    return tf_buffer_->lookupTransform(to_frame, from_frame, time_stamp, lookup_timeout_);
  }
  catch (tf2::TransformException& e) {
    ex = e;
  }

  // if that failed, try to get the newest one if requested
  if (retry_lookup_newest_) {
    try {
      return tf_buffer_->lookupTransform(to_frame, from_frame, rclcpp::Time(0, 0), lookup_timeout_);
    }
    catch (tf2::TransformException& e) {
      ex = e;
    }
  }

  // if the flow got here, we've failed to look the transform up
  if (quiet_) {
    RCLCPP_DEBUG(node_->get_logger(), "Transformer: Exception caught while looking up transform from \"%s\" to \"%s\": %s", from_frame.c_str(),
                 to_frame.c_str(), ex.what());
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Transformer: Exception caught while looking up transform from \"%s\" to \"%s\": %s",
                         from_frame.c_str(), to_frame.c_str(), ex.what());
  }

  return std::nullopt;
}

std::optional<geometry_msgs::msg::TransformStamped> Transformer::getTransformImpl(const std::string& from_frame, const rclcpp::Time& from_stamp,
                                                                                  const std::string& to_frame, const rclcpp::Time& to_stamp,
                                                                                  const std::string& fixed_frame, const std::string& latlon_frame) {
  if (!initialized_) {
    std::cout << "Transformer: cannot provide transform, not initialized" << std::endl;
    return std::nullopt;
  }

  // if the frames are the same, just return an identity transform
  if (from_frame == to_frame)
    return create_transform(from_frame, to_frame, to_stamp, tf2::toMsg(tf2::Transform::getIdentity()));

  // check for a transform from/to latlon coordinates - that is a special case handled separately
  if (from_frame == latlon_frame) {
    // find the transformation between the UTM frame and the non-latlon frame to fill the returned tf
    const std::string utm_frame = getFramePrefix(from_frame) + "utm_origin";
    auto              tf_opt    = getTransformImpl(utm_frame, from_stamp, to_frame, to_stamp, fixed_frame, latlon_frame);
    if (!tf_opt.has_value())
      return std::nullopt;
    // change the transformation frames to point from latlon
    frame_from(*tf_opt) = from_frame;
    return tf_opt;
  } else if (to_frame == latlon_frame) {
    // find the transformation between the UTM frame and the non-latlon frame to fill the returned tf
    const std::string utm_frame = getFramePrefix(to_frame) + "utm_origin";
    auto              tf_opt    = getTransformImpl(from_frame, from_stamp, utm_frame, to_stamp, fixed_frame, latlon_frame);
    if (!tf_opt.has_value())
      return std::nullopt;
    // change the transformation frames to point to latlon
    frame_to(*tf_opt) = to_frame;
    return tf_opt;
  }

  tf2::TransformException ex("");
  // first try to get transform at the requested time
  try {
    // try looking up and returning the transform
    return tf_buffer_->lookupTransform(to_frame, to_stamp, from_frame, from_stamp, fixed_frame, lookup_timeout_);
  }
  catch (tf2::TransformException& e) {
    ex = e;
  }

  // if that failed, try to get the newest one if requested
  if (retry_lookup_newest_) {
    try {
      return tf_buffer_->lookupTransform(to_frame, from_frame, rclcpp::Time(0, 0), lookup_timeout_);
    }
    catch (tf2::TransformException& e) {
      ex = e;
    }
  }

  // if the flow got here, we've failed to look the transform up
  if (quiet_) {
    RCLCPP_DEBUG(node_->get_logger(), "Transformer: Exception caught while looking up transform from \"%s\" to \"%s\": %s", from_frame.c_str(),
                 to_frame.c_str(), ex.what());
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Transformer: Exception caught while looking up transform from \"%s\" to \"%s\": %s",
                         from_frame.c_str(), to_frame.c_str(), ex.what());
  }

  return std::nullopt;
}

//}

/* resolveFrameImpl() //{*/

std::string Transformer::resolveFrameImpl(const std::string& frame_id) {
  // if the frame is empty, return the default frame id
  if (frame_id.empty())
    return default_frame_id_;

  // if there is no prefix set, just return the raw frame id
  if (prefix_.empty())
    return frame_id;

  // if there is a default prefix set and the frame does not start with it, prefix it
  if (frame_id.substr(0, prefix_.length()) != prefix_)
    return prefix_ + frame_id;

  return frame_id;
}

//}

/* LLtoUTM() method //{ */
geometry_msgs::msg::Point Transformer::LLtoUTM(const geometry_msgs::msg::Point& what, [[maybe_unused]] const std::string& prefix) {
  // convert LAT-LON to UTM
  geometry_msgs::msg::Point utm;
  mrs_lib::UTM(what.x, what.y, &utm.x, &utm.y);
  // copy the height from the input
  utm.z = what.z;
  return utm;
}

geometry_msgs::msg::PointStamped Transformer::LLtoUTM(const geometry_msgs::msg::PointStamped& what, const std::string& prefix) {
  geometry_msgs::msg::PointStamped ret;
  ret.header.frame_id = prefix + "utm_origin";
  ret.header.stamp    = what.header.stamp;
  ret.point           = LLtoUTM(what.point, prefix);
  return ret;
}

geometry_msgs::msg::Pose Transformer::LLtoUTM(const geometry_msgs::msg::Pose& what, const std::string& prefix) {
  geometry_msgs::msg::Pose ret;
  ret.position    = LLtoUTM(what.position, prefix);
  ret.orientation = what.orientation;
  return ret;
}

geometry_msgs::msg::PoseStamped Transformer::LLtoUTM(const geometry_msgs::msg::PoseStamped& what, const std::string& prefix) {
  geometry_msgs::msg::PoseStamped ret;
  ret.header.frame_id = prefix + "utm_origin";
  ret.header.stamp    = what.header.stamp;
  ret.pose            = LLtoUTM(what.pose, prefix);
  return ret;
}
//}

/* UTMtoLL() method //{ */
std::optional<geometry_msgs::msg::Point> Transformer::UTMtoLL(const geometry_msgs::msg::Point& what, [[maybe_unused]] const std::string& prefix) {
  // if no UTM zone was specified by the user, we don't know which one to use...
  if (!got_utm_zone_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "cannot transform to latlong, missing UTM zone (did you call setLatLon()?)");
    return std::nullopt;
  }

  // now apply the nonlinear transformation from UTM to LAT-LON
  geometry_msgs::msg::Point latlon;
  mrs_lib::UTMtoLL(what.y, what.x, utm_zone_.data(), latlon.x, latlon.y);
  latlon.z = what.z;
  return latlon;
}

std::optional<geometry_msgs::msg::PointStamped> Transformer::UTMtoLL(const geometry_msgs::msg::PointStamped& what, const std::string& prefix) {
  const auto opt = UTMtoLL(what.point, prefix);
  if (!opt.has_value())
    return std::nullopt;

  geometry_msgs::msg::PointStamped ret;
  ret.header.frame_id = prefix + LATLON_ORIGIN;
  ret.header.stamp    = what.header.stamp;
  ret.point           = opt.value();
  return ret;
}

std::optional<geometry_msgs::msg::Pose> Transformer::UTMtoLL(const geometry_msgs::msg::Pose& what, const std::string& prefix) {
  const auto opt = UTMtoLL(what.position, prefix);
  if (!opt.has_value())
    return std::nullopt;

  geometry_msgs::msg::Pose ret;
  ret.position    = opt.value();
  ret.orientation = what.orientation;
  return ret;
}

std::optional<geometry_msgs::msg::PoseStamped> Transformer::UTMtoLL(const geometry_msgs::msg::PoseStamped& what, const std::string& prefix) {
  const auto opt = UTMtoLL(what.pose, prefix);
  if (!opt.has_value())
    return std::nullopt;

  geometry_msgs::msg::PoseStamped ret;
  ret.header.frame_id = prefix + LATLON_ORIGIN;
  ret.header.stamp    = what.header.stamp;
  ret.pose            = opt.value();
  return ret;
}
//}

/* getFramePrefix() method //{ */
std::string Transformer::getFramePrefix(const std::string& frame_id) {
  const auto firstof = frame_id.find_first_of('/');
  if (firstof == std::string::npos)
    return "";
  else
    return frame_id.substr(0, firstof + 1);
}
//}

}  // namespace mrs_lib
