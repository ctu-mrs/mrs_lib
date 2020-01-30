// clang: MatousFormat

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

  std::string from_frame_resolved = resolveFrameName(what.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (from_frame_resolved == to_frame_resolved)
  {
    T ret = what;
    ret.header.frame_id = from_frame_resolved;
    return ret;
  }

  // get the transform
  const auto tf_opt = getTransform(from_frame_resolved, to_frame_resolved, what.header.stamp);
  if (!tf_opt.has_value())
    return std::nullopt;
  mrs_lib::TransformStamped tf = tf_opt.value();

  // do the transformation
  const auto result_opt = transform(tf, what);
  return result_opt;
}

//}

/* /1* transform() //{ *1/ */

template <typename Mat>
std::optional<typename Mat::PlainObject> Transformer::transformVecs(const mrs_lib::TransformStamped& tf, const Eigen::MatrixBase<Mat>& what)
{
  const typename Mat::PlainObject mat(what);
  return Transformer::transformImpl(tf, mat);
}

template <class T>
std::optional<T> Transformer::transform(const mrs_lib::TransformStamped& tf, const T& what)
{
  if (!is_initialized_)
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return std::nullopt;
  }

  if (tf.from() == tf.to())
  {
    T ret = what;
    ret.header.frame_id = tf.from();
    ret.header.stamp = tf.stamp();
    return ret;
  }

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
