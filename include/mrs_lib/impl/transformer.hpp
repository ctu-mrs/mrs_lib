#ifndef TRANSFORMER_HPP
#define TRANSFORMER_HPP

#ifndef TRANSFORMER_H
#include <mrs_lib/transformer.h>
#endif

namespace mrs_lib
{

// | --------------------- helper methods --------------------- |

/* getHeader() overloads for different message types (pointers, pointclouds etc) //{ */

template <typename msg_t>
std_msgs::msg::Header Transformer::getHeader(const msg_t& msg) {
  return msg.header;
}

#ifdef PCL_SPECIALIZATION
template <typename pt_t>
std_msgs::msg::Header Transformer::getHeader(const pcl::PointCloud<pt_t>& cloud) {
  std_msgs::msg::Header ret;
  pcl_conversions::fromPCL(cloud.header, ret);
  return ret;
}
#endif

//}

/* setHeader() overloads for different message types (pointers, pointclouds etc) //{ */

template <typename msg_t>
void Transformer::setHeader(msg_t& msg, const std_msgs::msg::Header& header) {
  msg.header = header;
}

#ifdef PCL_SPECIALIZATION
template <typename pt_t>
void Transformer::setHeader(pcl::PointCloud<pt_t>& cloud, const std_msgs::msg::Header& header) {
  pcl_conversions::toPCL(header, cloud.header);
}
#endif

//}

/* copyChangeFrame() helper function //{ */

template <typename T>
T Transformer::copyChangeFrame(const T& what, const std::string& frame_id) {
  T ret = what;
  if constexpr (has_header_member_v<T>) {
    std_msgs::msg::Header new_header = getHeader(what);
    new_header.frame_id              = frame_id;
    setHeader(ret, new_header);
  }
  return ret;
}

//}

/* transformImpl() //{ */

template <class T>
std::optional<T> Transformer::transformImpl(const geometry_msgs::msg::TransformStamped& tf, const T& what) {

  const std::string from_frame = frame_from(tf);
  const std::string to_frame   = frame_to(tf);

  if (from_frame == to_frame)
    return copyChangeFrame(what, from_frame);

  const std::string latlon_frame_name = resolveFrameImpl(LATLON_ORIGIN);

  // First, check if the transformation is from/to the latlon frame
  // if conversion between UVM and LatLon coordinates is defined for this message, it may be resolved
  if constexpr (UTMLL_exists_v<Transformer, T>) {
    // check for transformation from LAT-LON GPS
    if (from_frame == latlon_frame_name) {
      const std::optional<T> tmp = LLtoUTM(what, getFramePrefix(from_frame));
      if (!tmp.has_value())
        return std::nullopt;
      return doTransform(tmp.value(), tf);
    }
    // check for transformation to LAT-LON GPS
    else if (to_frame == latlon_frame_name) {
      const std::optional<T> tmp = doTransform(what, tf);
      if (!tmp.has_value())
        return std::nullopt;
      return UTMtoLL(tmp.value(), getFramePrefix(to_frame));
    }
  } else {
    // by default, transformation from/to LATLON is undefined, so return nullopt if it's attempted
    if (from_frame == latlon_frame_name || to_frame == latlon_frame_name) {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9,
                                   "Transformer: cannot transform message of this type (" << typeid(T).name() << ") to/from latitude/longitude coordinates!");
      return std::nullopt;
    }
  }

  // otherwise it's just an ol' borin' transformation
  return doTransform(what, tf);
}

//}

/* doTransform() //{ */

template <class T>
std::optional<T> Transformer::doTransform(const T& what, const geometry_msgs::msg::TransformStamped& tf) {
  try {
    T result;
    tf2::doTransform(what, result, tf);
    return result;
  }
  catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Transformer: Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s",
                         frame_from(tf).c_str(), frame_to(tf).c_str(), ex.what());
    return std::nullopt;
  }
}

//}

// | ------------------ user-callable methods ----------------- |

/* transformSingle() //{ */

template <class T>
std::optional<T> Transformer::transformSingle(const T& what, const std::string& to_frame_raw) {
  const std_msgs::msg::Header orig_header = getHeader(what);
  return transformSingle(orig_header.frame_id, what, to_frame_raw, orig_header.stamp);
}

template <class T>
std::optional<T> Transformer::transformSingle(const std::string& from_frame_raw, const T& what, const std::string& to_frame_raw,
                                              const rclcpp::Time& time_stamp) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Transformer: cannot transform, not initialized");
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

  return transformImpl(tf_resolved, what);
}

//}

/* transform() //{ */

template <class T>
std::optional<T> Transformer::transform(const T& what, const geometry_msgs::msg::TransformStamped& tf) {
  std::scoped_lock lck(mutex_);

  if (!initialized_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1e9, "Transformer: cannot transform, not initialized");
    return std::nullopt;
  }

  const std::string                          from_frame  = resolveFrameImpl(frame_from(tf));
  const std::string                          to_frame    = resolveFrameImpl(frame_to(tf));
  const geometry_msgs::msg::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);

  return transformImpl(tf_resolved, what);
}

/* //} */

}  // namespace mrs_lib

#endif  // TRANSFORMER_HPP
