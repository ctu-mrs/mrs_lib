#ifndef TRANSFORMER_HPP
#define TRANSFORMER_HPP

// clang: MatousFormat

/* getHeader() overloads for different message types (pointers, pointclouds etc) //{ */

template <typename msg_t>
std_msgs::Header getHeader(const msg_t& msg)
{
  return msg.header;
}

template <typename pt_t>
std_msgs::Header getHeader(const pcl::PointCloud<pt_t>& cloud)
{
  std_msgs::Header ret;
  pcl_conversions::fromPCL(cloud.header, ret);
  return ret;
}

//}

/* setHeader() overloads for different message types (pointers, pointclouds etc) //{ */

template <typename msg_t>
void setHeader(msg_t& msg, const std_msgs::Header& header)
{
  msg.header = header;
}

template <typename pt_t>
void setHeader(pcl::PointCloud<pt_t>& cloud, const std_msgs::Header& header)
{
  pcl_conversions::toPCL(header, cloud.header);
}

//}

/* copyWithHeader() helper function //{ */

namespace impl
{
  template<typename T>
  using _has_header_member_chk = decltype( std::declval<T&>().header );
  template<typename T>
  constexpr bool has_header_member_v = std::experimental::is_detected<_has_header_member_chk, T>::value;
}

template <typename T>
void copyChangeFrame(const T& what, const std::string& frame_id)
{
  T ret = what;
  if constexpr (impl::has_header_member_v<T>)
  {
    std_msgs::Header new_header = getHeader(what);
    new_header.frame_id = frame_id;
    setHeader(ret, new_header);
  }
  return ret;
}

//}

/* transformImpl() //{ */

template <class T>
std::optional<T> Transformer::transformImpl(const geometry_msgs::TransformStamped& tf, const T& what)
{
  const std::string from_frame = frame_from(tf);
  const std::string to_frame = frame_to(tf);

  if (from_frame == to_frame)
    return copyChangeFrame(what, from_frame);

  std::string latlon_frame_name = resolveFrame(LATLON_ORIGIN);
  // by default, transformation from/to LATLON is undefined
  if (frame_from(tf) == latlon_frame_name || frame_to(tf) == latlon_frame_name)
    return std::nullopt;
  return doTransform(tf, what);
}

//}

/* transformSingle() //{ */

template <class T>
std::optional<T> Transformer::transformSingle(const std::string& to_frame_raw, const T& what)
{
  if (!initialized_)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return std::nullopt;
  }

  const std_msgs::Header orig_header = getHeader(what);
  const std::string from_frame = resolveFrame(orig_header.frame_id);
  const std::string to_frame = resolveFrame(to_frame_raw);

  // get the transform
  const auto tf_opt = getTransform(from_frame, to_frame, orig_header.stamp);
  if (!tf_opt.has_value())
    return std::nullopt;
  const geometry_msgs::TransformStamped& tf = tf_opt.value();

  // do the transformation
  const auto result_opt = transform(tf, what);
  return result_opt;
}

//}

/* transform() //{ */

template <class T>
std::optional<T> Transformer::transform(const geometry_msgs::TransformStamped& tf, const T& what)
{
  if (!initialized_)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return std::nullopt;
  }

  const std::string from_frame = resolveFrame(frame_from(tf));
  const std::string to_frame = resolveFrame(frame_to(tf));
  const geometry_msgs::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);

  return transformImpl(tf_resolved, what);
}

/* //} */

/* doTransform() //{ */

template <class T>
std::optional<T> Transformer::doTransform(const geometry_msgs::TransformStamped& tf, const T& what)
{
  try
  {
    T result;
    tf2::doTransform(what, result, tf);
    return result;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transformer: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", node_name_.c_str(), frame_from(tf).c_str(),
                      frame_to(tf).c_str(), ex.what());
    return std::nullopt;
  }
}

//}

template <class T>
std::optional<T> Transformer::transformFromLatLon(const std::string& to_frame, const ros::Time& at_time, const std::string& uav_prefix, const T& what)
{
  return geometry::fromEigen(transformFromLatLon(to_frame, at_time, uav_prefix, geometry::toEigen(what)));
}

template <class T>
std::optional<T> Transformer::transformToLatLon(const std::string& from_frame, const ros::Time& at_time, const std::string& uav_prefix, const T& what)
{
  return geometry::fromEigen(transformToLatLon(from_frame, at_time, uav_prefix, geometry::toEigen(what)));
}

#endif // TRANSFORMER_HPP
