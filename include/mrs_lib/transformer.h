/**  \file Implements the Transformer class - wrapper for ROS's TF2 lookups and transformations.
 *   \brief A wrapper for easier work with tf2 transformations.
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
 *   \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#ifndef TRANSFORMER_H
#define TRANSFORMER_H

/* #define PCL_SPECIALIZATION true */
/* #define OPENCV_SPECIALIZATION true */

/* #define EIGEN_DONT_VECTORIZE */
/* #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT */

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mrs_msgs/msg/reference_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <std_msgs/msg/header.hpp>

#include <mrs_lib/geometry/misc.h>

#ifdef PCL_SPECIALIZATION
#include <pcl_ros/point_cloud.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#endif

#include <mutex>
#include <experimental/type_traits>

//}

#ifdef PCL_SPECIALIZATION

namespace tf2
{

template <typename pt_t>
void doTransform(const pcl::PointCloud<pt_t>& cloud_in, pcl::PointCloud<pt_t>& cloud_out, const geometry_msgs::msg::TransformStamped& transform) {
  pcl_ros::transformPointCloud(cloud_in, cloud_out, transform.transform);
  pcl_conversions::toPCL(transform.header.stamp, cloud_out.header.stamp);
  cloud_out.header.frame_id = transform.header.frame_id;
}

}  // namespace tf2

#endif

namespace mrs_lib
{

static const std::string UTM_ORIGIN    = "utm_origin";
static const std::string LATLON_ORIGIN = "latlon_origin";

/**
 * \brief A convenience wrapper class for ROS's native TF2 API to simplify transforming of various messages.
 *
 * Implements optional automatic frame prefix deduction, seamless transformation lattitude/longitude coordinates and UTM coordinates, simple transformation of
 * MRS messages etc.
 */
/* class Transformer //{ */

class Transformer {

public:
  /* Constructor, assignment operator and overloads //{ */

  /**
   * \brief A convenience constructor that doesn't initialize anything.
   *
   * This constructor is just to enable usign the Transformer as a member variable of nodelets etc.
   * To actually initialize the class, use the alternative constructor.
   *
   * \note This constructor doesn't initialize the TF2 transform listener and all calls to the transformation-related methods of an object constructed using
   * this method will fail.
   */
  Transformer();

  /**
   * \brief The main constructor that actually initializes stuff.
   *
   * This constructor initializes the class and the TF2 transform listener.
   *
   * \param node        the node handle to be used for subscribing to the transformations.
   */
  Transformer(const rclcpp::Node::SharedPtr& node);

  /**
   * \brief The main constructor that actually initializes stuff.
   *
   * This constructor initializes the class and the TF2 transform listener.
   *
   * \param node        the node handle to be used for subscribing to the transformations.
   * \param cache_time  duration of the transformation buffer's cache into the past that will be kept.
   */
  /* Transformer(const rclcpp::Node::SharedPtr& node, const rclcpp::Duration& cache_time = rclcpp::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME)); */

  /**
   * \brief The main constructor that actually initializes stuff.
   *
   * This constructor initializes the class and the TF2 transform listener.
   *
   * \param node        the node handle to be used for subscribing to the transformations.
   * \param cache_time  duration of the transformation buffer's cache into the past that will be kept.
   */
  Transformer(const rclcpp::Node::SharedPtr& node, const rclcpp::Clock::SharedPtr& clock,
              const rclcpp::Duration& cache_time = rclcpp::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), const rclcpp::QoS& qos = rclcpp::ServicesQoS());

  /**
   * \brief A convenience move assignment operator.
   *
   * This operator moves all data from the object that is being assigned from, invalidating it.
   *
   * \param  other  the object to assign from. It will be invalid after this method returns.
   * \return        a reference to the object being assigned to.
   */
  Transformer& operator=(Transformer&& other);

  //}

  // | ------------------ Configuration methods ----------------- |

  /* setDefaultFrame() //{ */

  /**
   * \brief Sets the default frame ID to be used instead of any empty frame ID.
   *
   * If you call e.g. the transform() method with a message that has an empty header.frame_id field, this value will be used instead.
   *
   * \param frame_id the default frame ID. Use an empty string to disable default frame ID deduction.
   *
   * \note Disabled by default.
   */
  void setDefaultFrame(const std::string& frame_id) {
    std::scoped_lock lck(mutex_);
    default_frame_id_ = frame_id;
  }

  //}

  /* setDefaultPrefix() //{ */

  /**
   * \brief Sets the default frame ID prefix to be used if no prefix is present in the frame.
   *
   * If you call any method with a frame ID that doesn't begin with this string, it will be automatically prefixed including a forward slash between the
   * prefix and raw frame ID. The forward slash should therefore *not* be included in the prefix.
   *
   * Example frame ID resolution assuming default prefix is "uav1":
   *   "local_origin" -> "uav1/local_origin"
   *
   * \param prefix the default frame ID prefix (without the forward slash at the end). Use an empty string to disable default frame ID prefixing.
   *
   * \note Disabled by default. The prefix will be applied as a namespace (with a forward slash between the prefix and raw frame ID).
   */
  void setDefaultPrefix(const std::string& prefix) {
    std::scoped_lock lck(mutex_);
    if (prefix.empty())
      prefix_ = "";
    else
      prefix_ = prefix + "/";
  }

  //}

  /* setLatLon() //{ */

  /**
   * \brief Sets the curret lattitude and longitude for UTM zone calculation.
   *
   * The Transformer uses this to deduce the current UTM zone used for transforming stuff to latlon_origin.
   *
   * \param lat the latitude in degrees.
   * \param lon the longitude in degrees.
   *
   * \note Any transformation to latlon_origin will fail if this function is not called first!
   */
  void setLatLon(const double lat, const double lon);

  //}

  /* setLookupTimeout() //{ */

  /**
   * \brief Set a timeout for transform lookup.
   *
   * The transform lookup operation will block up to this duration if the transformation is not available immediately.
   *
   * \note Disabled by default.
   *
   * \param timeout the lookup timeout. Set to zero to disable blocking.
   */
  void setLookupTimeout(const rclcpp::Duration timeout = rclcpp::Duration(0, 0)) {
    std::scoped_lock lck(mutex_);
    lookup_timeout_ = timeout;
  }

  //}

  /* retryLookupNewest() //{ */

  /**
   * \brief Enable/disable retry of a failed transform lookup with \p rclcpp::Time(0).
   *
   * If enabled, a failed transform lookup will be retried.
   * The new try will ignore the requested timestamp and will attempt to fetch the latest transformation between the two frames.
   *
   * \note Disabled by default.
   *
   * \param retry enables or disables retry.
   */
  void retryLookupNewest(const bool retry = true) {
    std::scoped_lock lck(mutex_);
    retry_lookup_newest_ = retry;
  }

  //}

  /* beQuiet() //{ */

  /**
   * \brief Enable/disable some prints that may be too noisy.
   *
   * \param quiet enables or disables quiet mode.
   *
   * \note Disabled by default.
   */
  void beQuiet(const bool quiet = true) {
    std::scoped_lock lck(mutex_);
    quiet_ = quiet;
  }

  //}

  /* resolveFrame() //{ */
  /**
   * \brief Deduce the full frame ID from a shortened or empty string using current default prefix and default frame rules.
   *
   * Example assuming default prefix is "uav1" and default frame is "uav1/gps_origin":
   *   "" -> "uav1/gps_origin"
   *   "local_origin" -> "uav1/local_origin"
   *
   * \param frame_id The frame ID to be resolved.
   *
   * \return The resolved frame ID.
   */
  std::string resolveFrame(const std::string& frame_id) {
    std::scoped_lock lck(mutex_);
    return resolveFrameImpl(frame_id);
  }
  //}

  /* transformSingle() //{ */

  /**
   * \brief Transforms a single variable to a new frame and returns it or \p std::nullopt if transformation fails.
   *
   * \param what the object to be transformed.
   * \param to_frame the target frame ID.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<T> transformSingle(const T& what, const std::string& to_frame);

  /**
   * \brief Transforms a single variable to a new frame.
   *
   * A convenience override for shared pointers.
   *
   * \param what The object to be transformed.
   * \param to_frame The target fram ID.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<std::shared_ptr<T>> transformSingle(const std::shared_ptr<T>& what, const std::string& to_frame) {
    return transformSingle(std::shared_ptr<const T>(what), to_frame);
  }

  /**
   * \brief Transforms a single variable to a new frame.
   *
   * A convenience override for shared pointers to const.
   *
   * \param what The object to be transformed.
   * \param to_frame The target fram ID.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<std::shared_ptr<T>> transformSingle(const std::shared_ptr<const T>& what, const std::string& to_frame) {
    auto ret = transformSingle(*what, to_frame);
    if (ret == std::nullopt)
      return std::nullopt;
    else
      return std::make_shared<T>(std::move(ret.value()));
  }

  /**
   * \brief Transforms a single variable to a new frame and returns it or \p std::nullopt if transformation fails.
   *
   * A convenience overload for headerless variables.
   *
   * \param from_frame the original target frame ID.
   * \param what the object to be transformed.
   * \param to_frame the target frame ID.
   * \param time_stamp the time of the transformation.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<T> transformSingle(const std::string& from_frame, const T& what, const std::string& to_frame,
                                                 const rclcpp::Time& time_stamp = rclcpp::Time(0));

  /**
   * \brief Transforms a single variable to a new frame and returns it or \p std::nullopt if transformation fails.
   *
   * A convenience overload for shared pointers to headerless variables.
   *
   * \param from_frame the original target frame ID.
   * \param what the object to be transformed.
   * \param to_frame the target frame ID.
   * \param time_stamp the time of the transformation.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<std::shared_ptr<T>> transformSingle(const std::string& from_frame, const std::shared_ptr<T>& what, const std::string& to_frame,
                                                                  const rclcpp::Time& time_stamp = rclcpp::Time(0)) {
    return transformSingle(from_frame, std::shared_ptr<const T>(what), to_frame, time_stamp);
  }

  /**
   * \brief Transforms a single variable to a new frame and returns it or \p std::nullopt if transformation fails.
   *
   * A convenience overload for shared pointers to const headerless variables.
   *
   * \param from_frame the original target frame ID.
   * \param what the object to be transformed.
   * \param to_frame the target frame ID.
   * \param time_stamp the time of the transformation.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<std::shared_ptr<T>> transformSingle(const std::string& from_frame, const std::shared_ptr<const T>& what,
                                                                  const std::string& to_frame, const rclcpp::Time& time_stamp = rclcpp::Time(0)) {
    auto ret = transformSingle(from_frame, *what, to_frame, time_stamp);
    if (ret == std::nullopt)
      return std::nullopt;
    else
      return std::make_shared<T>(std::move(ret.value()));
  }

  //}

  /* transform() //{ */

  /**
   * \brief Transform a variable to new frame using a particular transformation.
   *
   * \param tf The transformation to be used.
   * \param what The object to be transformed.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<T> transform(const T& what, const geometry_msgs::msg::TransformStamped& tf);

  /**
   * \brief Transform a variable to new frame using a particular transformation.
   *
   * A convenience override for shared pointers to const.
   *
   * \param tf The transformation to be used.
   * \param what The object to be transformed.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<std::shared_ptr<T>> transform(const std::shared_ptr<const T>& what, const geometry_msgs::msg::TransformStamped& tf) {
    auto ret = transform(*what, tf);
    if (ret == std::nullopt)
      return std::nullopt;
    else
      return std::make_shared<T>(std::move(ret.value()));
  }

  /**
   * \brief Transform a variable to new frame using a particular transformation.
   *
   * A convenience override for shared pointers.
   *
   * \param tf The transformation to be used.
   * \param what The object to be transformed.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  template <class T>
  [[nodiscard]] std::optional<std::shared_ptr<T>> transform(const std::shared_ptr<T>& what, const geometry_msgs::msg::TransformStamped& tf) {
    return transform(std::shared_ptr<const T>(what), tf);
  }

  //}

  /* transformAsVector() method //{ */
  /**
   * \brief Transform an Eigen::Vector3d (interpreting it as a vector).
   *
   * Only the rotation will be applied to the variable.
   *
   * \param tf The transformation to be used.
   * \param what The vector to be transformed.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  [[nodiscard]] std::optional<Eigen::Vector3d> transformAsVector(const Eigen::Vector3d& what, const geometry_msgs::msg::TransformStamped& tf);

  /**
   * \brief Transform an Eigen::Vector3d (interpreting it as a vector).
   *
   * Only the rotation will be applied to the variable.
   *
   * \param from_frame  The current frame of \p what.
   * \param what        The vector to be transformed.
   * \param to_frame    The desired frame of \p what.
   * \param time_stamp  From which time to take the transformation (use \p rclcpp::Time(0) for the latest time).
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  [[nodiscard]] std::optional<Eigen::Vector3d> transformAsVector(const std::string& from_frame, const Eigen::Vector3d& what, const std::string& to_frame,
                                                                 const rclcpp::Time& time_stamp = rclcpp::Time(0));
  //}

  /* transformAsPoint() method //{ */
  /**
   * \brief Transform an Eigen::Vector3d (interpreting it as a point).
   *
   * Both the rotation and translation will be applied to the variable.
   *
   * \param tf The transformation to be used.
   * \param what The object to be transformed.
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  [[nodiscard]] std::optional<Eigen::Vector3d> transformAsPoint(const Eigen::Vector3d& what, const geometry_msgs::msg::TransformStamped& tf);

  /**
   * \brief Transform an Eigen::Vector3d (interpreting it as a point).
   *
   * Both the rotation and translation will be applied to the variable.
   *
   * \param from_frame  The current frame of \p what.
   * \param what The object to be transformed.
   * \param to_frame    The desired frame of \p what.
   * \param time_stamp  From which time to take the transformation (use \p rclcpp::Time(0) for the latest time).
   *
   * \return \p std::nullopt if failed, optional containing the transformed object otherwise.
   */
  [[nodiscard]] std::optional<Eigen::Vector3d> transformAsPoint(const std::string& from_frame, const Eigen::Vector3d& what, const std::string& to_frame,
                                                                const rclcpp::Time& time_stamp = rclcpp::Time(0));
  //}

  /* getTransform() //{ */

  /**
   * \brief Obtains a transform between two frames in a given time.
   *
   * \param from_frame The original frame of the transformation.
   * \param to_frame The target frame of the transformation.
   * \param time_stamp The time stamp of the transformation. (0 will get the latest)
   *
   * \return \p std::nullopt if failed, optional containing the requested transformation otherwise.
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getTransform(const std::string& from_frame, const std::string& to_frame,
                                                                                 const rclcpp::Time& time_stamp = rclcpp::Time(0));

  /**
   * \brief Obtains a transform between two frames in a given time.
   *
   * This overload enables the user to select a different time of the source frame and the target frame.
   *
   * \param from_frame The original frame of the transformation.
   * \param from_stamp The time at which the original frame should be evaluated. (0 will get the latest)
   * \param to_frame The target frame of the transformation.
   * \param to_stamp The time to which the data should be transformed. (0 will get the latest)
   * \param fixed_frame The frame that may be assumed constant in time (the "world" frame).
   *
   * \return \p std::nullopt if failed, optional containing the requested transformation otherwise.
   *
   * \note An example of when this API may be useful is explained here: http://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28C%2B%2B%29
   */
  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getTransform(const std::string& from_frame, const rclcpp::Time& from_stamp,
                                                                                 const std::string& to_frame, const rclcpp::Time& to_stamp,
                                                                                 const std::string& fixed_frame);

  //}

private:
  /* private members, methods etc //{ */

  std::mutex mutex_;

  // keeps track whether a non-basic constructor was called and the transform listener is initialized
  bool                    initialized_ = false;
  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // user-configurable options
  std::string      default_frame_id_    = "";
  std::string      prefix_              = "";  // if not empty, includes the forward slash
  bool             quiet_               = false;
  rclcpp::Duration lookup_timeout_      = rclcpp::Duration(0, 0);
  bool             retry_lookup_newest_ = false;

  bool                 got_utm_zone_ = false;
  std::array<char, 10> utm_zone_     = {};

  // returns the first namespace prefix of the frame (if any) includin the forward slash
  std::string getFramePrefix(const std::string& frame_id);

  template <class T>
  std::optional<T>                               transformImpl(const geometry_msgs::msg::TransformStamped& tf, const T& what);
  std::optional<mrs_msgs::msg::ReferenceStamped> transformImpl(const geometry_msgs::msg::TransformStamped& tf, const mrs_msgs::msg::ReferenceStamped& what);
  std::optional<Eigen::Vector3d>                 transformImpl(const geometry_msgs::msg::TransformStamped& tf, const Eigen::Vector3d& what);

  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getTransformImpl(const std::string& from_frame, const std::string& to_frame,
                                                                                     const rclcpp::Time& time_stamp, const std::string& latlon_frame);

  [[nodiscard]] std::optional<geometry_msgs::msg::TransformStamped> getTransformImpl(const std::string& from_frame, const rclcpp::Time& from_stamp,
                                                                                     const std::string& to_frame, const rclcpp::Time& to_stamp,
                                                                                     const std::string& fixed_frame, const std::string& latlon_frame);

  std::string resolveFrameImpl(const std::string& frame_id);

  template <class T>
  std::optional<T> doTransform(const T& what, const geometry_msgs::msg::TransformStamped& tf);

  //}

  // | ------------------- some helper methods ------------------ |
public:
  /* frame_from(), frame_to() and inverse() methods //{ */

  /**
   * \brief A convenience function that returns the frame from which the message transforms.
   *
   * \param msg the message representing the transformation.
   * \return    the frame from which the transformation in \msg transforms.
   */
  static constexpr const std::string& frame_from(const geometry_msgs::msg::TransformStamped& msg) {
    return msg.child_frame_id;
  }

  /**
   * \brief A convenience function that returns the frame from which the message transforms.
   *
   * This overload returns a reference to the string in the message representing the frame id so that it can be modified.
   *
   * \param msg the message representing the transformation.
   * \return    a reference to the field in the message containing the string with the frame id from which the transformation in \msg transforms.
   */
  static constexpr std::string& frame_from(geometry_msgs::msg::TransformStamped& msg) {
    return msg.child_frame_id;
  }

  /**
   * \brief A convenience function that returns the frame to which the message transforms.
   *
   * \param msg the message representing the transformation.
   * \return    the frame to which the transformation in \msg transforms.
   */
  static constexpr const std::string& frame_to(const geometry_msgs::msg::TransformStamped& msg) {
    return msg.header.frame_id;
  }

  /**
   * \brief A convenience function that returns the frame to which the message transforms.
   *
   * This overload returns a reference to the string in the message representing the frame id so that it can be modified.
   *
   * \param msg the message representing the transformation.
   * \return    a reference to the field in the message containing the string with the frame id to which the transformation in \msg transforms.
   */
  static constexpr std::string& frame_to(geometry_msgs::msg::TransformStamped& msg) {
    return msg.header.frame_id;
  }

  /**
   * \brief A convenience function implements returns the inverse of the transform message as a one-liner.
   *
   * \param msg the message representing the transformation.
   * \return    a new message representing an inverse of the original transformation.
   */
  static geometry_msgs::msg::TransformStamped inverse(const geometry_msgs::msg::TransformStamped& msg) {
    tf2::Transform tf2;
    tf2::fromMsg(msg.transform, tf2);
    tf2 = tf2.inverse();
    return create_transform(frame_to(msg), frame_from(msg), msg.header.stamp, tf2::toMsg(tf2));
  }
  //}

private:
  /* create_transform() method //{ */
  static geometry_msgs::msg::TransformStamped create_transform(const std::string& from_frame, const std::string& to_frame, const rclcpp::Time& time_stamp) {
    geometry_msgs::msg::TransformStamped ret;
    frame_from(ret)  = from_frame;
    frame_to(ret)    = to_frame;
    ret.header.stamp = time_stamp;
    return ret;
  }

  static geometry_msgs::msg::TransformStamped create_transform(const std::string& from_frame, const std::string& to_frame, const rclcpp::Time& time_stamp,
                                                               const geometry_msgs::msg::Transform& tf) {
    geometry_msgs::msg::TransformStamped ret;
    frame_from(ret)  = from_frame;
    frame_to(ret)    = to_frame;
    ret.header.stamp = time_stamp;
    ret.transform    = tf;
    return ret;
  }
  //}

  /* copyChangeFrame() and related methods //{ */

  // helper type and member for detecting whether a message has a header using SFINAE
  template <typename T>
  using has_header_member_chk = decltype(std::declval<T&>().header);

  template <typename T>
  static constexpr bool has_header_member_v = std::experimental::is_detected<has_header_member_chk, T>::value;

  template <typename msg_t>
  std_msgs::msg::Header getHeader(const msg_t& msg);

#ifdef PCL_SPECIALIZATION
  template <typename pt_t>
  std_msgs::msg::Header getHeader(const pcl::PointCloud<pt_t>& cloud);

  template <typename pt_t>
  void setHeader(pcl::PointCloud<pt_t>& cloud, const std_msgs::msg::Header& header);
#endif

  template <typename msg_t>
  void setHeader(msg_t& msg, const std_msgs::msg::Header& header);

  template <typename T>
  T copyChangeFrame(const T& what, const std::string& frame_id);

  //}

  /* methods for converting between lattitude/longitude and UTM coordinates //{ */
  geometry_msgs::msg::Point        LLtoUTM(const geometry_msgs::msg::Point& what, const std::string& prefix);
  geometry_msgs::msg::PointStamped LLtoUTM(const geometry_msgs::msg::PointStamped& what, const std::string& prefix);
  geometry_msgs::msg::Pose         LLtoUTM(const geometry_msgs::msg::Pose& what, const std::string& prefix);
  geometry_msgs::msg::PoseStamped  LLtoUTM(const geometry_msgs::msg::PoseStamped& what, const std::string& prefix);

  std::optional<geometry_msgs::msg::Point>        UTMtoLL(const geometry_msgs::msg::Point& what, const std::string& prefix);
  std::optional<geometry_msgs::msg::PointStamped> UTMtoLL(const geometry_msgs::msg::PointStamped& what, const std::string& prefix);
  std::optional<geometry_msgs::msg::Pose>         UTMtoLL(const geometry_msgs::msg::Pose& what, const std::string& prefix);
  std::optional<geometry_msgs::msg::PoseStamped>  UTMtoLL(const geometry_msgs::msg::PoseStamped& what, const std::string& prefix);

  // helper types and member for detecting whether the UTMtoLL and LLtoUTM methods are defined for a certain message
  template <class Class, typename Message>
  using UTMLL_method_chk = decltype(std::declval<Class>().UTMtoLL(std::declval<const Message&>(), ""));

  template <class Class, typename Message>
  using LLUTM_method_chk = decltype(std::declval<Class>().LLtoUTM(std::declval<const Message&>(), ""));

  template <class Class, typename Message>
  static constexpr bool UTMLL_exists_v =
      std::experimental::is_detected<UTMLL_method_chk, Class, Message>::value && std::experimental::is_detected<LLUTM_method_chk, Class, Message>::value;
  //}
};

//}

}  // namespace mrs_lib

#ifndef TRANSFORMER_HPP
#include <mrs_lib/impl/transformer.hpp>
#endif

#endif  // TRANSFORMER_H
