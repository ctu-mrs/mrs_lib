// clang: MatousFormat

/**  \file Implements the Transformer class - wrapper for ROS's TF2 lookups and transformations.
 *   \brief A wrapper for easier work with tf2 transformations.
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
 *   \author Matouš Vrba - matous.vrba@fel.cvut.cz
 */
#ifndef TRANSFORMER_H
#define TRANSFORMER_H

/* #define EIGEN_DONT_VECTORIZE */
/* #define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT */

/* includes //{ */

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <mrs_msgs/ReferenceStamped.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Header.h>

#include <mrs_lib/geometry/misc.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>
#include <experimental/type_traits>

//}

namespace tf2
{

  template <typename pt_t>
  void doTransform(const pcl::PointCloud<pt_t>& cloud_in, pcl::PointCloud<pt_t>& cloud_out, const geometry_msgs::TransformStamped& transform)
  {
    pcl_ros::transformPointCloud(cloud_in, cloud_out, transform.transform);
    pcl_conversions::toPCL(transform.header.stamp, cloud_out.header.stamp);
    cloud_out.header.frame_id = transform.header.frame_id;
  }

}  // namespace tf2

namespace mrs_lib
{

  static const std::string UTM_ORIGIN = "utm_origin";
  static const std::string LATLON_ORIGIN = "latlon_origin";

  /**
   * \brief A convenience wrapper class for ROS's native TF2 API to simplify transforming of various messages.
   *
   * Implements optional automatic frame prefix deduction, seamless transformation lattitude/longitude coordinates and UTM coordinates, simple transformation of MRS messages etc.
   */
  /* class Transformer //{ */

  class Transformer
  {

  public:
    /* Constructor and overloads //{ */

    /**
     * \brief A convenience constructor that doesn't initialize anything.
     *
     * This constructor is just to enable usign the Transformer as a member variable of nodelets etc.
     * To actually initialize the class, use the alternative constructor.
     *
     * \note This constructor doesn't initialize the TF2 transform listener and all calls to the transformation-related methods of an object constructed using this method will fail.
     */
    Transformer();

    /**
     * @brief The main constructor that actually initializes stuff.
     *
     * This constructor initializes the class and the TF2 transform listener.
     *
     * @param node_name the name of the node running the transformer, is used in ROS prints. If you don't care, just set it to an empty string.
     */
    Transformer(const ros::NodeHandle& nh, const std::string& node_name = std::string());

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
    void setDefaultFrame(const std::string& frame_id)
    {
      default_frame_id_ = frame_id;
    }

    //}

    /* setDefaultPrefix() //{ */

    /**
     * \brief Sets the default frame ID prefix to be used if no prefix is present in the frame.
     *
     * If you call any method with a frame ID that doesn't begin with this string, it will be automatically prefixed including a forward slash between the prefix and raw frame ID.
     * The forward slash should therefore *not* be included in the prefix.
     *
     * Example frame ID resolution assuming default prefix is "uav1":
     *   "local_origin" -> "uav1/local_origin"
     *
     * \param prefix the default frame ID prefix (without the forward slash at the end). Use an empty string to disable default frame ID prefixing.
     *
     * \note Disabled by default. The prefix will be applied as a namespace (with a forward slash between the prefix and raw frame ID).
     */
    void setDefaultPrefix(const std::string& prefix)
    {
      prefix_ = prefix;
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
    void setLookupTimeout(const ros::Duration timeout = ros::Duration(0))
    {
      lookup_timeout_ = timeout;
    }

    //}

    /* retryLookupNewest() //{ */

    /**
     * \brief Enable/disable retry of a failed transform lookup with \p ros::Time(0).
     *
     * If enabled, a failed transform lookup will be retried.
     * The new try will ignore the requested timestamp and will attempt to fetch the latest transformation between the two frames.
     *
     * \note Disabled by default.
     *
     * \param retry enables or disables retry.
     */
    void retryLookupNewest(const bool retry = true)
    {
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
    void beQuiet(const bool quiet = true)
    {
      quiet_ = quiet;
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
    [[nodiscard]] std::optional<T> transformSingle(const std::string& from_frame_raw, const T& what, const std::string& to_frame_raw, const ros::Time& time_stamp = ros::Time(0));

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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transformSingle(const boost::shared_ptr<const T>& what, const std::string& to_frame)
    {
      auto ret = transformSingle(*what, to_frame);
      if (ret == std::nullopt)
        return std::nullopt;
      else
        return boost::make_shared<T>(std::move(ret.value()));
    }

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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transformSingle(const boost::shared_ptr<T>& what, const std::string& to_frame)
    {
      return transformSingle(boost::shared_ptr<const T>(what), to_frame);
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
    [[nodiscard]] std::optional<T> transform(const T& what, const geometry_msgs::TransformStamped& tf);

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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform(const boost::shared_ptr<const T>& what, const geometry_msgs::TransformStamped& tf)
    {
      auto ret = transform(*what, tf);
      if (ret == std::nullopt)
        return std::nullopt;
      else
        return boost::make_shared<T>(std::move(ret.value()));
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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform(const boost::shared_ptr<T>& what, const geometry_msgs::TransformStamped& tf)
    {
      return transform(boost::shared_ptr<const T>(what), tf);
    }

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
    [[nodiscard]] std::optional<geometry_msgs::TransformStamped> getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& time_stamp = ros::Time(0));

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
    [[nodiscard]] std::optional<geometry_msgs::TransformStamped> getTransform(const std::string& from_frame, const ros::Time& from_stamp, const std::string& to_frame, const ros::Time& to_stamp, const std::string& fixed_frame);

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
    std::string resolveFrame(const std::string& frame_id);
    //}

  private:
    /* private members, methods etc //{ */

    /**
     * \brief keeps track whether a non-basic constructor was called and the transform listener is initialized
     */
    bool initialized_ = false;
    std::string node_name_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

    // user-configurable options
    std::string default_frame_id_ = "";
    std::string prefix_ = "";
    bool quiet_ = false;
    ros::Duration lookup_timeout_ = ros::Duration(0);
    bool retry_lookup_newest_ = false;

    bool got_utm_zone_ = false;
    char utm_zone_[10] = {};

    std::string getFramePrefix(const std::string& frame_id);

    template <class T>
    std::optional<T> transformImpl(const geometry_msgs::TransformStamped& tf, const T& what);
    std::optional<mrs_msgs::ReferenceStamped> transformImpl(const geometry_msgs::TransformStamped& tf, const mrs_msgs::ReferenceStamped& what);
    std::optional<Eigen::Vector3d> transformImpl(const geometry_msgs::TransformStamped& tf, const Eigen::Vector3d& what);

    template <class T>
    std::optional<T> doTransform(const T& what, const geometry_msgs::TransformStamped& tf);

    //}

    // | ------------------- some helper methods ------------------ |
  public:
    /* frame_from(), frame_to() and inverse() methods //{ */
    
    static constexpr const std::string& frame_from(const geometry_msgs::TransformStamped& msg)
    {
      return msg.header.frame_id;
    }
    
    static constexpr std::string& frame_from(geometry_msgs::TransformStamped& msg)
    {
      return msg.header.frame_id;
    }

    static constexpr const std::string& frame_to(const geometry_msgs::TransformStamped& msg)
    {
      return msg.child_frame_id;
    }

    static constexpr std::string& frame_to(geometry_msgs::TransformStamped& msg)
    {
      return msg.child_frame_id;
    }

    static geometry_msgs::TransformStamped inverse(const geometry_msgs::TransformStamped& msg)
    {
      tf2::Transform tf2;
      tf2::fromMsg(msg.transform, tf2);
      tf2 = tf2.inverse();
      return create_transform(msg.child_frame_id, msg.header.frame_id, msg.header.stamp, tf2::toMsg(tf2));
    }
    //}

  private:
    /* create_transform() method //{ */
    static geometry_msgs::TransformStamped create_transform(const std::string& from_frame, const std::string& to_frame, const ros::Time& time_stamp)
    {
      geometry_msgs::TransformStamped ret;
      frame_from(ret) = from_frame;
      frame_to(ret) = to_frame;
      ret.header.stamp = time_stamp;
      return ret;
    }

    static geometry_msgs::TransformStamped create_transform(const std::string& from_frame, const std::string& to_frame, const ros::Time& time_stamp, const geometry_msgs::Transform& tf)
    {
      geometry_msgs::TransformStamped ret;
      frame_from(ret) = from_frame;
      frame_to(ret) = to_frame;
      ret.header.stamp = time_stamp;
      ret.transform = tf;
      return ret;
    }
    //}

    /* copyChangeFrame() and related methods //{ */

    // helper type and member for detecting whether a message has a header using SFINAE
    template<typename T>
    using has_header_member_chk = decltype( std::declval<T&>().header );
    template<typename T>
    static constexpr bool has_header_member_v = std::experimental::is_detected<has_header_member_chk, T>::value;
    
    template <typename msg_t>
    std_msgs::Header getHeader(const msg_t& msg);
    template <typename pt_t>
    std_msgs::Header getHeader(const pcl::PointCloud<pt_t>& cloud);
    
    template <typename msg_t>
    void setHeader(msg_t& msg, const std_msgs::Header& header);
    template <typename pt_t>
    void setHeader(pcl::PointCloud<pt_t>& cloud, const std_msgs::Header& header);
    
    template <typename T>
    T copyChangeFrame(const T& what, const std::string& frame_id);
    
    //}

    /* methods for converting between lattitude/longitude and UTM coordinates //{ */
    Eigen::Vector3d LLtoUTM(const Eigen::Vector3d& what, [[maybe_unused]] const std::string& prefix);
    geometry_msgs::Point LLtoUTM(const geometry_msgs::Point& what, const std::string& prefix);
    geometry_msgs::Pose LLtoUTM(const geometry_msgs::Pose& what, const std::string& prefix);
    geometry_msgs::PoseStamped LLtoUTM(const geometry_msgs::PoseStamped& what, const std::string& prefix);
    
    std::optional<Eigen::Vector3d> UTMtoLL(const Eigen::Vector3d& what, [[maybe_unused]] const std::string& prefix);
    std::optional<geometry_msgs::Point> UTMtoLL(const geometry_msgs::Point& what, const std::string& prefix);
    std::optional<geometry_msgs::Pose> UTMtoLL(const geometry_msgs::Pose& what, const std::string& prefix);
    std::optional<geometry_msgs::PoseStamped> UTMtoLL(const geometry_msgs::PoseStamped& what, const std::string& prefix);
    
    // helper types and member for detecting whether the UTMtoLL and LLtoUTM methods are defined for a certain message
    template<typename T>
    using UTMLL_method_chk = decltype(Transformer().UTMtoLL(std::declval<const T&>(), ""));
    template<typename T>
    using LLUTM_method_chk = decltype(Transformer().LLtoUTM(std::declval<const T&>(), ""));
    template<typename T>
    static constexpr bool UTMLL_exists_v = std::experimental::is_detected<UTMLL_method_chk, T>::value && std::experimental::is_detected<LLUTM_method_chk, T>::value;
    //}

    /* methods for converting between Eigen and geometry_msgs types //{ */
    geometry_msgs::Quaternion fromEigen(const Eigen::Quaterniond& what);
    geometry_msgs::Point fromEigen(const Eigen::Vector3d& what);
    
    Eigen::Vector3d toEigen(const geometry_msgs::Point& what);
    Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion& what);
    //}

  };

#include <impl/transformer.hpp>

  //}

}  // namespace mrs_lib

#endif  // TRANSFORMER_H
