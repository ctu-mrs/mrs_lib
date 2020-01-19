
/* transformSingle() //{ */

template <class T>
std::optional<T> Transformer::transformSingle(const std::string& to_frame, const T& what)
{
  if (!is_initialized_) {
    ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", ros::this_node::getName().c_str());
    return std::nullopt;
  }

  std::string from_frame_resolved  = resolveFrameName(what.header.frame_id);
  std::string to_frame_resolved = resolveFrameName(to_frame);

  if (from_frame_resolved == to_frame_resolved )
  {
    T ret = what;
    ret.header.frame_id = from_frame_resolved;
    return ret;
  }

  mrs_lib::TransformStamped tf;

  // get the transform
  const auto tf_opt = getTransform(from_frame_resolved, to_frame_resolved, what.header.stamp);
  if (!tf_opt.has_value())
    return std::nullopt;

  // do the transformation
  const auto result_opt = transform(tf, what);
  return result_opt;
}

template <class T>
bool Transformer::transformSingle(const std::string& to_frame, const T& what, T& output)
{
  const auto result_opt = transformSingle(to_frame, what);
  if (result_opt.has_value())
  {
    output = result_opt.value();
    return true;
  }
  else
  {
    return false;
  }
}

//}

/* transformImpl() //{ */

template <class T>
std::optional<T> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const T& what)
{
  const auto tmp_result = prepareMessage(what);
  const auto result_opt = doTransform(tf, tmp_result);
  return postprocessMessage(result_opt.value());
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

  std::string from_frame_resolved  = resolveFrameName(what.header.frame_id);

  if (from_frame_resolved == tf.to())
  {
    T ret = what;
    ret.header.frame_id = from_frame_resolved;
    ret.header.stamp = tf.getTransform().header.stamp;
    return ret;
  }

  const auto result_opt = transformImpl(tf, what);
  return result_opt;
}

template <class T>
bool Transformer::transform(const mrs_lib::TransformStamped& to_frame, const T& what, T& output)
{
  const auto result_opt = transformSingle(to_frame, what);
  if (result_opt.has_value())
  {
    output = result_opt.value();
    return true;
  }
  else
  {
    return false;
  }
}

//}

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

/* prepareMessage() //{ */

template <class T>
T Transformer::prepareMessage(const T& what)
{
  return what;
}

//}

/* postprocessMessage() //{ */

template <class T>
T Transformer::postprocessMessage(const T& what)
{
  return what;
}

//}
