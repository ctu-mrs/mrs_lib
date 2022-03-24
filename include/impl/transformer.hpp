#ifndef TRANSFORMER_HPP
#define TRANSFORMER_HPP

// clang: MatousFormat

namespace mrs_lib
{

  // | --------------------- helper methods --------------------- |

  /* getHeader() overloads for different message types (pointers, pointclouds etc) //{ */

  template <typename msg_t>
  std_msgs::Header Transformer::getHeader(const msg_t& msg)
  {
    return msg.header;
  }

  template <typename pt_t>
  std_msgs::Header Transformer::getHeader(const pcl::PointCloud<pt_t>& cloud)
  {
    std_msgs::Header ret;
    pcl_conversions::fromPCL(cloud.header, ret);
    return ret;
  }

  //}

  /* setHeader() overloads for different message types (pointers, pointclouds etc) //{ */

  template <typename msg_t>
  void Transformer::setHeader(msg_t& msg, const std_msgs::Header& header)
  {
    msg.header = header;
  }

  template <typename pt_t>
  void Transformer::setHeader(pcl::PointCloud<pt_t>& cloud, const std_msgs::Header& header)
  {
    pcl_conversions::toPCL(header, cloud.header);
  }

  //}

  /* copyChangeFrame() helper function //{ */

  template <typename T>
  T Transformer::copyChangeFrame(const T& what, const std::string& frame_id)
  {
    T ret = what;
    if constexpr (has_header_member_v<T>)
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

    const std::string latlon_frame_name = resolveFrameImpl(LATLON_ORIGIN);

    // First, check if the transformation is from/to the latlon frame
    // if conversion between UVM and LatLon coordinates is defined for this message, it may be resolved
    if constexpr (UTMLL_exists_v<Transformer, T>)
    {
      // check for transformation from LAT-LON GPS
      if (from_frame == latlon_frame_name)
      {
        const std::optional<T> tmp = LLtoUTM(what, getFramePrefix(from_frame));
        if (!tmp.has_value())
          return std::nullopt;
        return doTransform(tmp.value(), tf);
      }
      // check for transformation to LAT-LON GPS
      else if (to_frame == latlon_frame_name)
      {
        const std::optional<T> tmp = doTransform(what, tf);
        if (!tmp.has_value())
          return std::nullopt;
        return UTMtoLL(tmp.value(), getFramePrefix(to_frame));
      }
    }
    else
    {
      // by default, transformation from/to LATLON is undefined, so return nullopt if it's attempted
      if (from_frame == latlon_frame_name || to_frame == latlon_frame_name)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[" << node_name_ << "]: Transformer: cannot transform message of this type (" << typeid(T).name() << ") to/from latitude/longitude coordinates!");
        return std::nullopt;
      }
    }

    // otherwise it's just an ol' borin' transformation
    return doTransform(what, tf);
  }

  //}

  /* doTransform() //{ */

  template <class T>
  std::optional<T> Transformer::doTransform(const T& what, const geometry_msgs::TransformStamped& tf)
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

  // | ------------------ user-callable methods ----------------- |

  /* transformSingle() //{ */

  template <class T>
  std::optional<T> Transformer::transformSingle(const T& what, const std::string& to_frame_raw)
  {
    const std_msgs::Header orig_header = getHeader(what);
    return transformSingle(orig_header.frame_id, what, to_frame_raw, orig_header.stamp);
  }

  template <class T>
  std::optional<T> Transformer::transformSingle(const std::string& from_frame_raw, const T& what, const std::string& to_frame_raw, const ros::Time& time_stamp)
  {
    std::scoped_lock lck(mutex_);

    if (!initialized_)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", node_name_.c_str());
      return std::nullopt;
    }

    const std::string from_frame = resolveFrameImpl(from_frame_raw);
    const std::string to_frame = resolveFrameImpl(to_frame_raw);
    const std::string latlon_frame = resolveFrameImpl(LATLON_ORIGIN);

    // get the transform
    const auto tf_opt = getTransformImpl(from_frame, to_frame, time_stamp, latlon_frame);
    if (!tf_opt.has_value())
      return std::nullopt;
    const geometry_msgs::TransformStamped& tf = tf_opt.value();

    // do the transformation
    const geometry_msgs::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);
    return transformImpl(tf_resolved, what);
  }

  //}

  /* transform() //{ */

  template <class T>
  std::optional<T> Transformer::transform(const T& what, const geometry_msgs::TransformStamped& tf)
  {
    std::scoped_lock lck(mutex_);

    if (!initialized_)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Transformer: cannot transform, not initialized", node_name_.c_str());
      return std::nullopt;
    }

    const std::string from_frame = resolveFrameImpl(frame_from(tf));
    const std::string to_frame = resolveFrameImpl(frame_to(tf));
    const geometry_msgs::TransformStamped tf_resolved = create_transform(from_frame, to_frame, tf.header.stamp, tf.transform);

    return transformImpl(tf_resolved, what);
  }

  /* //} */

}

#endif // TRANSFORMER_HPP
