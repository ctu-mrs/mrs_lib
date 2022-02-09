// clang: MatousFormat

#include <mrs_lib/transformer.h>
#include <mrs_lib/gps_conversions.h>

using test_t = mrs_msgs::ReferenceStamped;
/* using test_t = pcl::PointCloud<pcl::PointXYZ>; */
template std::optional<test_t> mrs_lib::Transformer::transform<test_t>(const mrs_lib::TransformStamped& to_frame, const test_t& what);
template std::optional<test_t::Ptr> mrs_lib::Transformer::transform<test_t>(const mrs_lib::TransformStamped& to_frame, const test_t::Ptr& what);
template std::optional<test_t::Ptr> mrs_lib::Transformer::transform<test_t>(const mrs_lib::TransformStamped& to_frame, const test_t::ConstPtr& what);
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
    : tf_listener_ptr_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_, node_name)), initialized_(true)
  {
  }

  //}

  /* transformImpl() //{ */

  std::optional<mrs_msgs::ReferenceStamped> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const mrs_msgs::ReferenceStamped& what)
  {
    const auto tmp_result = prepareMessage(what);
    const auto result_opt = transformImpl(tf, tmp_result);
    if (result_opt.has_value())
      return postprocessMessage(result_opt.value());
    else
      return std::nullopt;
  }

  std::optional<Eigen::Vector3d> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const Eigen::Vector3d& what)
  {
    const Eigen::Vector3d output = tf.getTransformEigen().rotation() * what;
    return output;
  }

  std::optional<geometry_msgs::PoseStamped> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const geometry_msgs::PoseStamped& what)
  {
    geometry_msgs::PoseStamped ret = what;
    std::string latlon_frame_name = resolveFrameName(LATLON_ORIGIN);

    // check for transformation from LAT-LON GPS
    if (tf.from() == latlon_frame_name)
    {
      // convert LAT-LON to UTM
      double utm_x, utm_y;
      mrs_lib::UTM(ret.pose.position.x, ret.pose.position.y, &utm_x, &utm_y);

      // utm_x and utm_y are now in 'utm_origin' frame
      const std::string uav_prefix = getUAVFramePrefix(tf.from());
      const std::string utm_frame_name = uav_prefix + "/utm_origin";
      ret.header.frame_id = utm_frame_name;
      ret.pose.position.x = utm_x;
      ret.pose.position.y = utm_y;

      // transform from 'utm_origin' to the desired frame
      const auto utm_origin_to_end_tf_opt = getTransform(utm_frame_name, tf.to(), tf.stamp());
      if (!utm_origin_to_end_tf_opt.has_value())
        return std::nullopt;
      return doTransform(utm_origin_to_end_tf_opt.value(), ret);
    }
    // check for transformation to LAT-LON GPS
    else if (tf.to() == latlon_frame_name)
    {
      // if no UTM zone was specified by the user, we don't know which one to use...
      if (!got_utm_zone_)
      {
        ROS_WARN_THROTTLE(1.0, "[%s]: cannot transform to latlong, missing UTM zone (did you call setCurrentLatLon()?)", node_name_.c_str());
        return std::nullopt;
      }

      // first, transform from the desired frame to 'utm_origin'
      const std::string uav_prefix = getUAVFramePrefix(tf.to());
      std::string utm_frame_name = uav_prefix + "/utm_origin";

      const auto start_to_utm_origin_tf_opt = getTransform(tf.from(), utm_frame_name, tf.stamp());

      if (!start_to_utm_origin_tf_opt.has_value())
        return std::nullopt;

      {
        const auto pose_opt = doTransform(start_to_utm_origin_tf_opt.value(), ret);

        if (!pose_opt.has_value())
          return std::nullopt;

        ret = std::move(pose_opt.value());
      }

      // now apply the nonlinear transformation from UTM to LAT-LON
      {
        std::scoped_lock lock(mutex_utm_zone_);
        double lat, lon;

        mrs_lib::UTMtoLL(ret.pose.position.y, ret.pose.position.x, utm_zone_, lat, lon);

        ret.header.frame_id = latlon_frame_name;
        ret.pose.position.x = lat;
        ret.pose.position.y = lon;

        return {std::move(ret)};
      }
    }
    // if nothing interesting is requested, just do the plain old linear transformation
    else
    {
      return doTransform(tf, ret);
    }
  }

  std::optional<geometry_msgs::Point> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const geometry_msgs::Point& what)
  {
    geometry_msgs::Point ret = what;
    std::string latlon_frame_name = resolveFrameName(LATLON_ORIGIN);

    // check for transformation from LAT-LON GPS
    if (tf.from() == latlon_frame_name)
    {
      // convert LAT-LON to UTM
      double utm_x, utm_y;
      mrs_lib::UTM(ret.x, ret.y, &utm_x, &utm_y);

      // utm_x and utm_y are now in 'utm_origin' frame
      const std::string uav_prefix = getUAVFramePrefix(tf.from());
      const std::string utm_frame_name = uav_prefix + "/utm_origin";
      ret.x = utm_x;
      ret.y = utm_y;

      // transform from 'utm_origin' to the desired frame
      const auto utm_origin_to_end_tf_opt = getTransform(utm_frame_name, tf.to(), tf.stamp());
      if (!utm_origin_to_end_tf_opt.has_value())
        return std::nullopt;
      return doTransform(utm_origin_to_end_tf_opt.value(), ret);
    }
    // check for transformation to LAT-LON GPS
    else if (tf.to() == latlon_frame_name)
    {
      // if no UTM zone was specified by the user, we don't know which one to use...
      if (!got_utm_zone_)
      {
        ROS_WARN_THROTTLE(1.0, "[%s]: cannot transform to latlong, missing UTM zone (did you call setCurrentLatLon()?)", node_name_.c_str());
        return std::nullopt;
      }

      // first, transform from the desired frame to 'utm_origin'
      const std::string uav_prefix = getUAVFramePrefix(tf.to());
      std::string utm_frame_name = uav_prefix + "/utm_origin";

      const auto start_to_utm_origin_tf_opt = getTransform(tf.from(), utm_frame_name, tf.stamp());

      if (!start_to_utm_origin_tf_opt.has_value())
        return std::nullopt;

      {
        const auto pose_opt = doTransform(start_to_utm_origin_tf_opt.value(), ret);

        if (!pose_opt.has_value())
          return std::nullopt;

        ret = std::move(pose_opt.value());
      }

      // now apply the nonlinear transformation from UTM to LAT-LON
      {
        std::scoped_lock lock(mutex_utm_zone_);
        double lat, lon;

        mrs_lib::UTMtoLL(ret.y, ret.x, utm_zone_, lat, lon);
        ret.x = lat;
        ret.y = lon;

        return {std::move(ret)};
      }
    }
    // if nothing interesting is requested, just do the plain old linear transformation
    else
    {
      return doTransform(tf, ret);
    }
  }

  //}

  /* prepareMessage() //{ */

  geometry_msgs::PoseStamped Transformer::prepareMessage(const mrs_msgs::ReferenceStamped& what)
  {
    // create the pose message
    geometry_msgs::PoseStamped pose;
    pose.header = what.header;

    pose.pose.position.x = what.reference.position.x;
    pose.pose.position.y = what.reference.position.y;
    pose.pose.position.z = what.reference.position.z;

    pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, what.reference.heading);

    return pose;
  }

  //}

  /* postprocessMessage() //{ */

  mrs_msgs::ReferenceStamped Transformer::postprocessMessage(const geometry_msgs::PoseStamped& what)
  {
    mrs_msgs::ReferenceStamped ret;
    ret.header = what.header;

    // copy the new transformed data back
    ret.reference.position.x = what.pose.position.x;
    ret.reference.position.y = what.pose.position.y;
    ret.reference.position.z = what.pose.position.z;

    try
    {
      ret.reference.heading = mrs_lib::AttitudeConverter(what.pose.orientation).getHeading();
    }
    catch (const mrs_lib::AttitudeConverter::GetHeadingException& e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s]: exception caught while transforming mrs_msgs::Reference's heading: %s", node_name_.c_str(), e.what());
      return std::nullopt;
    }

    return ret;
  }

  //}

  /* getTransform() //{ */

  std::optional<mrs_lib::TransformStamped> Transformer::getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& time_stamp,
                                                                     const bool quiet)
  {
    if (!is_initialized_)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot provide transform, not initialized", ros::this_node::getName().c_str());
      return std::nullopt;
    }

    const std::string to_frame_resolved = resolveFrameName(to_frame);
    const std::string from_frame_resolved = resolveFrameName(from_frame);
    const std::string latlon_frame_resolved = resolveFrameName(LATLON_ORIGIN);

    // check for latlon transform
    if (from_frame_resolved == latlon_frame_resolved || to_frame_resolved == latlon_frame_resolved)
      return mrs_lib::TransformStamped(from_frame_resolved, to_frame_resolved, time_stamp);

    // first try to get transform at the requested time
    try
    {
      std::scoped_lock lock(mutex_tf_buffer_);

      // get the transform
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(to_frame_resolved, from_frame_resolved, time_stamp);

      // return it
      return mrs_lib::TransformStamped(from_frame_resolved, to_frame_resolved, time_stamp, transform);
    }
    catch (tf2::TransformException& ex)
    {
      // this happens often -> DEBUG
      ROS_DEBUG("[%s]: Transformer: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(), from_frame_resolved.c_str(),
                to_frame_resolved.c_str(), ex.what());
    }

    // otherwise get the newest one
    try
    {
      std::scoped_lock lock(mutex_tf_buffer_);

      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(to_frame_resolved, from_frame_resolved, ros::Time(0));

      // return it
      return mrs_lib::TransformStamped(from_frame_resolved, to_frame_resolved, transform.header.stamp, transform);
    }
    catch (tf2::TransformException& ex)
    {
      if (quiet)
      {
        ROS_DEBUG("[%s]: Transformer: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(), from_frame_resolved.c_str(),
                  to_frame_resolved.c_str(), ex.what());
      } else
      {
        ROS_WARN_THROTTLE(1.0, "[%s]: Transformer: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(),
                          from_frame_resolved.c_str(), to_frame_resolved.c_str(), ex.what());
      }
    }

    return std::nullopt;
  }

  /* bool Transformer::getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& time_stamp, mrs_lib::TransformStamped& output)
   */
  /* { */
  /*   const auto result_opt = getTransform(from_frame, to_frame, time_stamp); */
  /*   if (result_opt.has_value()) */
  /*   { */
  /*     output = result_opt.value(); */
  /*     return true; */
  /*   } */
  /*   else */
  /*   { */
  /*     return false; */
  /*   } */
  /* } */

  //}

  /* resolve_frame() //{ */

  std::string Transformer::resolve_frame(const std::string& frame_id)
  {
    if (frame_id.empty())
      return default_frame_id_;

    // if there is a default prefix set and the frame does not start with it, prefix it
    if (!prefix_.empty() && frame_id.substr(0, prefix_.length()) != prefix_)
      return prefix_ + "/" + frame_id;

    return frame_id;
  }

  //}

  /* set_lat_lon() //{ */

  void set_lat_lon(const double lat, const double lon)
  {
    double utm_x, utm_y;
    LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_);
  }

  //}

}  // namespace mrs_lib
