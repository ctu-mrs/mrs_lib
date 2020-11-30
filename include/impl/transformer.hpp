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

template <typename msg_t>
std_msgs::Header getHeader(const boost::shared_ptr<msg_t>& msg)
{
  return getHeader(*msg);
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

template <typename msg_t>
void setHeader(boost::shared_ptr<msg_t>& msg, const std_msgs::Header& header)
{
  setHeader(*msg, header);
}

//}

template <typename T>
T copyWithHeader(const T& what, const std::string& frame_id, const ros::Time& stamp)
{
  T ret = what;
  std_msgs::Header new_header;
  new_header.frame_id = frame_id;
  new_header.stamp = stamp;
  setHeader(ret, new_header);
  return ret;
}

template <typename T>
boost::shared_ptr<T> copyWithHeader(const boost::shared_ptr<const T>& what, const std::string& frame_id, const ros::Time& stamp)
{
  return boost::make_shared<T>(std::move(copyWithHeader(*what, frame_id, stamp)));
}

template <typename T>
boost::shared_ptr<T> copyWithHeader(const boost::shared_ptr<T>& what, const std::string& frame_id, const ros::Time& stamp)
{
  return copyWithHeader(boost::const_pointer_cast<const T>(what), frame_id, stamp);
}

/* transformImpl() //{ */

template <class T>
std::optional<T> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const T& what)
{
  std::string latlon_frame_name = resolveFrameName(LATLON_ORIGIN);
  // by default, transformation from/to LATLON is undefined
  if (tf.from() == latlon_frame_name || tf.to() == latlon_frame_name)
    return std::nullopt;
  return doTransform(tf, what);
}

//}

/* transformSingle() //{ */

template <class T>
std::optional<T> Transformer::transformSingle(const std::string& to_frame, const T& what)
{
  if (!is_initialized_)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return std::nullopt;
  }

  const std_msgs::Header orig_header = getHeader(what);
  const std::string from_frame_resolved = resolveFrameName(orig_header.frame_id);
  const std::string to_frame_resolved = resolveFrameName(to_frame);

  if (from_frame_resolved == to_frame_resolved)
    return copyWithHeader(what, from_frame_resolved, orig_header.stamp);

  // get the transform
  const auto tf_opt = getTransform(from_frame_resolved, to_frame_resolved, orig_header.stamp);
  if (!tf_opt.has_value())
    return std::nullopt;
  mrs_lib::TransformStamped tf = tf_opt.value();

  // do the transformation
  const auto result_opt = transform(tf, what);
  return result_opt;
}

//}

/* transform() //{ */

template <class T>
std::optional<T> Transformer::transform(const mrs_lib::TransformStamped& tf, const T& what)
{
  if (!is_initialized_)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return std::nullopt;
  }

  if (tf.from() == tf.to())
    return copyWithHeader(what, tf.from(), tf.stamp());

  const auto result_opt = transformImpl(tf, what);
  return result_opt;
}

/* //} */

/* transformHeaderless() //{ */

template <class T>
std::optional<T> Transformer::transformHeaderless(const mrs_lib::TransformStamped& tf, const T& what)
{
  if (!is_initialized_)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return std::nullopt;
  }

  if (tf.from() == tf.to())
    return what;

  const auto result_opt = transformImpl(tf, what);
  return result_opt;
}

/* //} */

/* doTransform() //{ */

template <class T>
std::optional<T> Transformer::doTransform(const mrs_lib::TransformStamped& tf, const T& what)
{
  try
  {
    T result;
    geometry_msgs::TransformStamped transform = tf.getTransform();
    tf2::doTransform(what, result, transform);
    return result;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "[%s]: Transformer: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", node_name_.c_str(), tf.from().c_str(),
                      tf.to().c_str(), ex.what());
    return std::nullopt;
  }
}

//}
