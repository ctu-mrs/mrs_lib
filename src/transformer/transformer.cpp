// clang: MatousFormat

#include <mrs_lib/transformer.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/geometry/conversions.h>

using test_t = mrs_msgs::ReferenceStamped;
/* using test_t = pcl::PointCloud<pcl::PointXYZ>; */
template std::optional<test_t> mrs_lib::Transformer::transform<test_t>(const geometry_msgs::TransformStamped& to_frame, const test_t& what);
template std::optional<test_t::Ptr> mrs_lib::Transformer::transform<test_t>(const geometry_msgs::TransformStamped& to_frame, const test_t::Ptr& what);
template std::optional<test_t::Ptr> mrs_lib::Transformer::transform<test_t>(const geometry_msgs::TransformStamped& to_frame, const test_t::ConstPtr& what);
template std::optional<test_t> mrs_lib::Transformer::transformSingle<test_t>(const std::string& to_frame, const test_t& what);
template std::optional<test_t::Ptr> mrs_lib::Transformer::transformSingle<test_t>(const std::string& to_frame, const test_t::Ptr& what);
template std::optional<test_t::Ptr> mrs_lib::Transformer::transformSingle<test_t>(const std::string& to_frame, const test_t::ConstPtr& what);

namespace mrs_lib
{

  // | ----------------------- Transformer ---------------------- |

  /* Transformer() (constructors)//{ */

  Transformer::Transformer()
  {
  }

  Transformer::Transformer(const std::string& node_name)
    : initialized_(true), tf_listener_ptr_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_, node_name)), node_name_(node_name)
  {
  }

  //}

  /* transformImpl() //{ */

  /* specialization for mrs_msgs::ReferenceStamped //{ */
  
  std::optional<mrs_msgs::ReferenceStamped> Transformer::transformImpl(const geometry_msgs::TransformStamped& tf, const mrs_msgs::ReferenceStamped& what)
  {
    // create a pose message
    geometry_msgs::PoseStamped pose;
    pose.header = what.header;
  
    pose.pose.position.x = what.reference.position.x;
    pose.pose.position.y = what.reference.position.y;
    pose.pose.position.z = what.reference.position.z;
    pose.pose.orientation = geometry::fromEigen(geometry::quaternionFromHeading(what.reference.heading));
  
    // try to transform the pose message
    const auto pose_opt = transformImpl(tf, pose);
    if (!pose_opt.has_value())
      return std::nullopt;
    // overwrite the pose with it's transformed value
    pose = pose_opt.value();
  
    mrs_msgs::ReferenceStamped ret;
    ret.header = pose.header;
  
    // copy the new transformed data back
    ret.reference.position.x = pose.pose.position.x;
    ret.reference.position.y = pose.pose.position.y;
    ret.reference.position.z = pose.pose.position.z;
    ret.reference.heading = geometry::headingFromRot(geometry::toEigen(pose.pose.orientation));
    return ret;
  }
  
  //}

  /* specialization for Eigen::Vector3d //{ */
  
  std::optional<Eigen::Vector3d> Transformer::transformImpl(const geometry_msgs::TransformStamped& tf, const Eigen::Vector3d& what)
  {
    // just transform it as you would a geometry_msgs::Point
    const auto opt = transformImpl(tf, geometry::fromEigen(what));
    if (opt.has_value())
      return geometry::toEigen(opt.value());
    else
      return std::nullopt;
  }
  
  //}

  //}

  /* getTransform() //{ */

  std::optional<geometry_msgs::TransformStamped> Transformer::getTransform(const std::string& from_frame_raw, const std::string& to_frame_raw, const ros::Time& time_stamp)
  {
    if (!initialized_)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot provide transform, not initialized", ros::this_node::getName().c_str());
      return std::nullopt;
    }

    const std::string from_frame = resolveFrame(from_frame_raw);
    const std::string to_frame = resolveFrame(to_frame_raw);
    const std::string latlon_frame = resolveFrame(LATLON_ORIGIN);

    // if the frames are the same, just return an identity transform
    if (from_frame == to_frame)
      return create_transform(from_frame, to_frame, time_stamp, tf2::toMsg(tf2::Transform::getIdentity()));

    // check for a transform from/to latlon coordinates - that is a special case handled separately
    if (from_frame == latlon_frame)
    {
      // find the transformation between the UTM frame and the non-latlon frame to fill the returned tf
      const std::string utm_frame = getFramePrefix(from_frame) + "/utm_origin";
      auto tf_opt = getTransform(utm_frame, to_frame, time_stamp);
      if (!tf_opt.has_value())
        return std::nullopt;
      // change the transformation frames to point from latlon
      frame_from(*tf_opt) = from_frame;
      return tf_opt;
    }
    else if (to_frame == latlon_frame)
    {
      // find the transformation between the UTM frame and the non-latlon frame to fill the returned tf
      const std::string utm_frame = getFramePrefix(to_frame) + "/utm_origin";
      auto tf_opt = getTransform(from_frame, utm_frame, time_stamp);
      if (!tf_opt.has_value())
        return std::nullopt;
      // change the transformation frames to point to latlon
      frame_to(*tf_opt) = to_frame;
      return tf_opt;
    }

    tf2::TransformException ex("");
    // first try to get transform at the requested time
    try
    {
      // try looking up and returning the transform
      return tf_buffer_.lookupTransform(from_frame, to_frame, time_stamp);
    }
    catch (tf2::TransformException& e)
    {
      ex = e;
    }

    // if that failed, try to get the newest one if requested
    if (retry_lookup_newest_)
    {
      try
      {
        return tf_buffer_.lookupTransform(from_frame, to_frame, ros::Time(0));
      }
      catch (tf2::TransformException& e)
      {
        ex = e;
      }
    }

    // if the flow got here, we've failed to look the transform up
    if (quiet_)
    {
      ROS_DEBUG("[%s]: Transformer: Exception caught while looking up transform from \"%s\" to \"%s\": %s", node_name_.c_str(), from_frame.c_str(),
                to_frame.c_str(), ex.what());
    } else
    {
      ROS_WARN_THROTTLE(1.0, "[%s]: Transformer: Exception caught while looking up transform from \"%s\" to \"%s\": %s", node_name_.c_str(),
                        from_frame.c_str(), to_frame.c_str(), ex.what());
    }

    return std::nullopt;
  }

  //}

  /* resolveFrame() //{ */

  std::string Transformer::resolveFrame(const std::string& frame_id)
  {
    if (frame_id.empty())
      return default_frame_id_;

    // if there is a default prefix set and the frame does not start with it, prefix it
    if (!prefix_.empty() && frame_id.substr(0, prefix_.length()) != prefix_)
      return prefix_ + "/" + frame_id;

    return frame_id;
  }

  //}

  /* setLatLon() //{ */

  void Transformer::setLatLon(const double lat, const double lon)
  {
    double utm_x, utm_y;
    mrs_lib::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_);
  }

  //}

  /* LLtoUTM() method //{ */
  Eigen::Vector3d Transformer::LLtoUTM(const Eigen::Vector3d& what, [[maybe_unused]] const std::string& prefix)
  {
    // convert LAT-LON to UTM
    Eigen::Vector3d utm;
    mrs_lib::UTM(what.x(), what.y(), &(utm.x()), &(utm.y()));
    // copy the height from the input
    utm.z() = what.z();
    return utm;
  }

  geometry_msgs::Point Transformer::LLtoUTM(const geometry_msgs::Point& what, const std::string& prefix)
  {
    return geometry::fromEigen(LLtoUTM(geometry::toEigen(what), prefix));
  }

  geometry_msgs::Pose Transformer::LLtoUTM(const geometry_msgs::Pose& what, const std::string& prefix)
  {
    geometry_msgs::Pose ret;
    ret.position = LLtoUTM(what.position, prefix);
    ret.orientation = what.orientation;
    return ret;
  }

  geometry_msgs::PoseStamped Transformer::LLtoUTM(const geometry_msgs::PoseStamped& what, const std::string& prefix)
  {
    geometry_msgs::PoseStamped ret;
    ret.header.frame_id = prefix + "/utm_origin";
    ret.header.stamp = what.header.stamp;
    ret.pose = LLtoUTM(what.pose, prefix);
    return ret;
  }
  //}

  /* UTMtoLL() method //{ */
  std::optional<Eigen::Vector3d> Transformer::UTMtoLL(const Eigen::Vector3d& what, [[maybe_unused]] const std::string& prefix)
  {
    // if no UTM zone was specified by the user, we don't know which one to use...
    if (!got_utm_zone_)
    {
      ROS_WARN_THROTTLE(1.0, "[%s]: cannot transform to latlong, missing UTM zone (did you call setLatLon()?)", node_name_.c_str());
      return std::nullopt;
    }
  
    // now apply the nonlinear transformation from UTM to LAT-LON
    Eigen::Vector3d latlon;
    mrs_lib::UTMtoLL(what.y(), what.x(), utm_zone_, latlon.x(), latlon.y());
    latlon.z() = what.z();
    return latlon;
  }

  std::optional<geometry_msgs::Point> Transformer::UTMtoLL(const geometry_msgs::Point& what, const std::string& prefix)
  {
    const auto opt = UTMtoLL(geometry::toEigen(what), prefix);
    if (!opt.has_value())
      return std::nullopt;

    return geometry::fromEigen(opt.value());
  }

  std::optional<geometry_msgs::Pose> Transformer::UTMtoLL(const geometry_msgs::Pose& what, const std::string& prefix)
  {
    const auto opt = UTMtoLL(what.position, prefix);
    if (!opt.has_value())
      return std::nullopt;

    geometry_msgs::Pose ret;
    ret.position = opt.value();
    ret.orientation = what.orientation;
    return ret;
  }

  std::optional<geometry_msgs::PoseStamped> Transformer::UTMtoLL(const geometry_msgs::PoseStamped& what, const std::string& prefix)
  {
    const auto opt = UTMtoLL(what.pose, prefix);
    if (!opt.has_value())
      return std::nullopt;

    geometry_msgs::PoseStamped ret;
    ret.header.frame_id = prefix + "/utm_origin";
    ret.header.stamp = what.header.stamp;
    ret.pose = opt.value();
    return ret;
  }
  //}

  std::string Transformer::getFramePrefix(const std::string& frame_id)
  {
    return frame_id.substr(0, frame_id.find_first_of('/'));
  }

}  // namespace mrs_lib
