// clang: MatousFormat

#include <mrs_lib/transformer.h>
#include <mrs_lib/GpsConversions.h>

using test_t = mrs_msgs::ReferenceStamped;
template std::optional<test_t> mrs_lib::Transformer::transform<test_t>(const mrs_lib::TransformStamped& to_frame, const test_t& what);
template std::optional<test_t> mrs_lib::Transformer::transformSingle<test_t>(const std::string& to_frame, const test_t& what);

namespace mrs_lib
{

  // | ----------------------- Transformer ---------------------- |

  /* Transformer() (constructors)//{ */

  Transformer::Transformer()
  {
  }

  Transformer::Transformer(const std::string& node_name, const std::string& uav_name)
  {

    this->node_name_ = node_name;
    this->uav_name_ = uav_name;

    if (uav_name == "")
    {
      this->got_uav_name_ = false;
    } else
    {
      this->got_uav_name_ = true;
    }

    this->current_control_frame_ = "";

    this->tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, node_name);

    this->is_initialized_ = true;
  }

  Transformer::Transformer(const std::string& node_name) : Transformer(node_name, ""){};

  Transformer::Transformer(const Transformer& other)
  {

    this->is_initialized_ = other.is_initialized_;
    this->node_name_ = other.node_name_;
    this->uav_name_ = other.uav_name_;
    this->got_uav_name_ = other.got_uav_name_;

    {
      std::scoped_lock lock(other.mutex_current_control_frame_);

      this->current_control_frame_ = other.current_control_frame_;
      this->got_current_control_frame_ = other.got_current_control_frame_;
    }

    if (this->is_initialized_)
    {
      this->tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this->node_name_);
    }
  }

  Transformer& Transformer::operator=(const Transformer& other)
  {

    if (this == &other)
    {
      return *this;
    }

    this->is_initialized_ = other.is_initialized_;
    this->node_name_ = other.node_name_;
    this->uav_name_ = other.uav_name_;
    this->got_uav_name_ = other.got_uav_name_;

    {
      std::scoped_lock lock(other.mutex_current_control_frame_);

      this->current_control_frame_ = other.current_control_frame_;
      this->got_current_control_frame_ = other.got_current_control_frame_;
    }

    if (this->is_initialized_)
    {
      this->tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this->node_name_);
    }

    return *this;
  }

  //}

  /* /1* transform() //{ *1/ */

  /* std::optional<Eigen::MatrixXd> Transformer::transform(const mrs_lib::TransformStamped& tf, const Eigen::MatrixXd& what) */
  /* { */
  /*   return transformImpl(tf, what); */
  /* } */


  /* //} */

  /* transformImpl() //{ */

  /* /1* Eigen::MatrixXd //{ *1/ */

  /* std::optional<Eigen::MatrixXd> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const Eigen::MatrixXd& what) */
  /* { */
  /*   if (what.rows() == 2) */
  /*   { */
  /*     const std::optional<Eigen::MatrixXd> tmp = transformMat2(tf, what); */
  /*     if (tmp.has_value()) */
  /*       return tmp.value(); */
  /*     else */
  /*       return std::nullopt; */
  /*   } */
  /*   else if (what.rows() == 3) */
  /*   { */
  /*     const std::optional<Eigen::MatrixXd> tmp = transformMat3(tf, what); */
  /*     if (tmp.has_value()) */
  /*       return tmp.value(); */
  /*     else */
  /*       return std::nullopt; */
  /*   } */
  /*   else */
  /*   { */
  /*     ROS_ERROR_THROTTLE(1.0, "[%s]: transformation of a %ldx%ld matrix is not implemented!", node_name_.c_str(), what.rows(), what.cols()); */
  /*     return std::nullopt; */
  /*   } */
  /* } */

  /* //} */

  /* /1* Eigen::Matrix<double, 2, -1> //{ *1/ */

  /* std::optional<Eigen::MatrixXd> Transformer::transformMat2(const mrs_lib::TransformStamped& tf, const Eigen::MatrixXd& what) */
  /* { */
  /*   assert(what.rows() == 2); */
  /*   Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(3, what.cols()); */
  /*   mat.block(0, 0, 2, what.cols()) = what; */
  /*   const std::optional<Eigen::MatrixXd> tmp = transformMat3(tf, mat); */
  /*   if (tmp.has_value()) */
  /*     return tmp.value().block(0, 0, 2, what.cols()); */
  /*   else */
  /*     return std::nullopt; */
  /* } */

  /* //} */

  /* /1* Eigen::Matrix<double, 3, -1> //{ *1/ */

  /* std::optional<Eigen::MatrixXd> Transformer::transformMat3(const mrs_lib::TransformStamped& tf, const Eigen::MatrixXd& what) */
  /* { */
  /*   assert(what.rows() == 3); */
  /*   Eigen::MatrixXd ret(what.rows(), what.cols()); */
  /*   std::string latlon_frame_name = resolveFrameName(LATLON_ORIGIN); */
  /*   Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::DontAlign> mat(what); */
  /*   /1* mat = what; *1/ */

  /*   // check for transformation from LAT-LON GPS */
  /*   /1* transformation from LAT-LON GPS //{ *1/ */

  /*   if (tf.from() == latlon_frame_name) */
  /*   { */
  /*     // utm_x and utm_y are now in 'utm_origin' frame */
  /*     const std::string uav_prefix = getUAVFramePrefix(tf.from()); */
  /*     const std::string utm_frame_name = uav_prefix + "/utm_origin"; */

  /*     // transform from 'utm_origin' to the desired frame */
  /*     const auto utm_origin_to_end_tf_opt = getTransform(utm_frame_name, tf.to(), tf.stamp()); */
  /*     if (!utm_origin_to_end_tf_opt.has_value()) */
  /*       return std::nullopt; */
  /*     const Eigen::Affine3d tf_eig = utm_origin_to_end_tf_opt.value().getTransformEigen(); */

  /*     for (int it = 0; it < ret.cols(); it++) */
  /*     { */
  /*       Eigen::Vector3d vec = mat.col(it); */
  /*       // convert LAT-LON to UTM */
  /*       mrs_lib::UTM(vec.x(), vec.y(), &vec.x(), &vec.y()); */
  /*       // transform to the desired target frame */
  /*       ret.col(it) = tf_eig*vec; */
  /*     } */
  /*   } */

  /*   //} */
  /*   // check for transformation to LAT-LON GPS */
  /*   /1* transformation to LAT-LON GPS //{ *1/ */

  /*   else if (tf.to() == latlon_frame_name) */
  /*   { */
  /*     std::string utm_zone; */
  /*     { */
  /*       std::scoped_lock lock(mutex_utm_zone_); */
  /*       // if no UTM zone was specified by the user, we don't know which one to use... */
  /*       if (!got_utm_zone_) */
  /*       { */
  /*         ROS_WARN_THROTTLE(1.0, "[%s]: cannot transform to latlong, missing UTM zone (did you call setCurrentLatLon()?)", node_name_.c_str()); */
  /*         return std::nullopt; */
  /*       } */
  /*       utm_zone = utm_zone_; */
  /*     } */

  /*     // first, transform from the desired frame to 'utm_origin' */
  /*     const std::string uav_prefix = getUAVFramePrefix(tf.to()); */
  /*     std::string utm_frame_name = uav_prefix + "/utm_origin"; */

  /*     const auto start_to_utm_origin_tf_opt = getTransform(tf.from(), utm_frame_name, tf.stamp()); */
  /*     if (!start_to_utm_origin_tf_opt.has_value()) */
  /*       return std::nullopt; */
  /*     const Eigen::Affine3d tf_eig = start_to_utm_origin_tf_opt.value().getTransformEigen(); */

  /*     // transform to the intermediate (UTM) target frame */
  /*     mat = tf_eig*mat; */
  /*     for (int it = 0; it < ret.cols(); it++) */
  /*     { */
  /*       Eigen::Vector3d vec = mat.col(it); */
  /*       // now apply the nonlinear transformation from UTM to LAT-LON */
  /*       mrs_lib::UTMtoLL(vec.y(), vec.x(), utm_zone, vec.x(), vec.y()); */
  /*       ret.col(it) = vec; */
  /*     } */
  /*   } */

  /*   //} */
  /*   // in case of a normal transformation just transform each vector separately */
  /*   /1* regular transformation //{ *1/ */

  /*   else */
  /*   { */
  /*     const Eigen::Affine3d tf_eig = tf.getTransformEigen(); */
  /*     ret = tf_eig*mat; */
  /*   } */

  /*   //} */
  /*   return ret; */
  /* } */

  /* //} */

  std::optional<mrs_msgs::ReferenceStamped> Transformer::transformImpl(const mrs_lib::TransformStamped& tf, const mrs_msgs::ReferenceStamped& what)
  {
    const auto tmp_result = prepareMessage(what);
    const auto result_opt = transformImpl(tf, tmp_result);
    if (result_opt.has_value())
      return postprocessMessage(result_opt.value());
    else
      return std::nullopt;
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

      const auto pose_opt = doTransform(start_to_utm_origin_tf_opt.value(), ret);

      if (!pose_opt.has_value())
        return std::nullopt;

      ret = pose_opt.value();

      // now apply the nonlinear transformation from UTM to LAT-LON
      {
        std::scoped_lock lock(mutex_utm_zone_);
        double lat, lon;

        mrs_lib::UTMtoLL(ret.pose.position.y, ret.pose.position.x, utm_zone_, lat, lon);

        ret.header.frame_id = latlon_frame_name;
        ret.pose.position.x = lat;
        ret.pose.position.y = lon;

        return ret;
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

      const auto pose_opt = doTransform(start_to_utm_origin_tf_opt.value(), ret);

      if (!pose_opt.has_value())
        return std::nullopt;

      ret = pose_opt.value();

      // now apply the nonlinear transformation from UTM to LAT-LON
      {
        std::scoped_lock lock(mutex_utm_zone_);
        double lat, lon;

        mrs_lib::UTMtoLL(ret.y, ret.x, utm_zone_, lat, lon);
        ret.x = lat;
        ret.y = lon;

        return ret;
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
    catch (mrs_lib::AttitudeConverter::GetHeadingException e)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s]: exception caught while transforming mrs_msgs::Reference's heading: %s", node_name_.c_str(), e.what());
      throw mrs_lib::Transformer::TransformException();
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
      return mrs_lib::TransformStamped(from_frame_resolved, to_frame_resolved, ros::Time::now(), transform);
    }
    catch (tf2::TransformException& ex)
    {
      if (quiet)
      {
        ROS_WARN_THROTTLE(1.0, "[%s]: Transformer: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(),
                          from_frame_resolved.c_str(), to_frame_resolved.c_str(), ex.what());
      } else
      {
        ROS_DEBUG("[%s]: Transformer: Exception caught while constructing transform from '%s' to '%s': %s", node_name_.c_str(), from_frame_resolved.c_str(),
                  to_frame_resolved.c_str(), ex.what());
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

  /* resolveFrameName() //{ */

  std::string Transformer::resolveFrameName(const std::string& in)
  {

    if (in == "")
    {

      if (got_current_control_frame_)
      {

        std::scoped_lock lock(mutex_current_control_frame_);

        return current_control_frame_;

      } else
      {

        ROS_WARN_THROTTLE(
            1.0, "[%s]: Transformer: could not resolve an empty frame_id, missing the current control frame (are you calling the setCurrentControlFrame()?)",
            node_name_.c_str());

        return "";
      }
    }

    if (in.substr(0, 3) != "uav")
    {

      if (got_uav_name_)
      {

        return uav_name_ + "/" + in;

      } else
      {

        ROS_WARN_THROTTLE(1.0, "[%s]: Transformer: could not deduce a namespaced frame_id '%s' (did you instance the Transformer with the uav_name argument?)",
                          node_name_.c_str(), in.c_str());
      }
    }

    return in;
  }

  //}

  /* checkUAVFramePrefix() //{ */

  std::string Transformer::getUAVFramePrefix(const std::string& in)
  {
    if (in.substr(0, 3) != "uav")
      std::string();
    const auto slashpos = in.find("/", 3);
    if (slashpos == std::string::npos)
      return in;
    else
      return in.substr(0, slashpos);
  }

  //}

  /* setCurrentControlFrame() //{ */

  void Transformer::setCurrentControlFrame(const std::string& in)
  {

    std::scoped_lock lock(mutex_current_control_frame_);

    current_control_frame_ = in;

    got_current_control_frame_ = true;
  }

  //}

  /* setCurrentLatLon() //{ */

  void Transformer::setCurrentLatLon(const double lat, const double lon)
  {

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

  TransformStamped::TransformStamped()
  {
  }

  TransformStamped::TransformStamped(const std::string from_frame, const std::string to_frame, ros::Time stamp)
  {

    this->from_frame_ = from_frame;
    this->to_frame_ = to_frame;
    this->stamp_ = stamp;
  }

  TransformStamped::TransformStamped(const std::string from_frame, const std::string to_frame, ros::Time stamp,
                                     const geometry_msgs::TransformStamped transform_stamped)
  {

    this->from_frame_ = from_frame;
    this->to_frame_ = to_frame;
    this->transform_stamped_ = transform_stamped;
    this->stamp_ = stamp;
  }

  //}

  /* from() //{ */

  std::string TransformStamped::from(void) const
  {

    return from_frame_;
  }

  //}

  /* to() //{ */

  std::string TransformStamped::to(void) const
  {

    return to_frame_;
  }

  //}

  /* stamp() //{ */

  ros::Time TransformStamped::stamp(void) const
  {

    return stamp_;
  }

  //}

  /* inverse() //{ */

  TransformStamped TransformStamped::inverse(void) const
  {
    geometry_msgs::TransformStamped tf = getTransform();
    tf2::Transform tf2;
    tf2::fromMsg(tf.transform, tf2);
    tf2 = tf2.inverse();
    tf.transform = tf2::toMsg(tf2);
    std::swap(tf.header.frame_id, tf.child_frame_id);
    TransformStamped ret(to(), from(), stamp(), tf);
    return ret;
  }

  //}

  /* getTransform() //{ */

  geometry_msgs::TransformStamped TransformStamped::getTransform(void) const
  {

    return transform_stamped_;
  }

  //}

  /* getTransformEigen() //{ */

  Eigen::Affine3d TransformStamped::getTransformEigen(void) const
  {
    return tf2::transformToEigen(getTransform().transform);
  }

  //}
}  // namespace mrs_lib
