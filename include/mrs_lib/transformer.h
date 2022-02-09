// clang: MatousFormat

/**  \file
 *   \brief A wrapper for easier work with tf2 transformations.
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
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
   * @brief tf wrapper that simplifies transforming stuff, adding caching, frame_id deduction and simple functions for one-time execution without getting the
   * transform first
   */
  /* class Transformer //{ */

  class Transformer
  {

  public:
    /* Constructor and overloads //{ */

    /**
     * @brief the basic constructor
     */
    Transformer();

    /**
     * @brief the constructor
     *
     * @param node_name the name of the node running the transformer, is used in ROS prints
     */
    Transformer(const std::string& node_name);

    //}

    /* setDefaultFrame() //{ */

    /**
     * @brief Sets the default frame ID to be used instead of any empty frame ID.
     *
     * @param The default frame ID.
     */
    void setDefaultFrame(const std::string& frame_id)
    {
      default_frame_id_ = frame_id;
    }

    //}

    /* setDefaultPrefix() //{ */

    /**
     * @brief Sets the default frame ID prefix to be used if no prefix is present in the frame.
     *
     * @param The default frame ID prefix.
     */
    void setDefaultPrefix(const std::string& prefix)
    {
      prefix_ = prefix;
    }

    //}

    /* setLatLon() //{ */

    /**
     * @brief Sets the curret lattitude and longitude for UTM zone calculation.
     *
     * The transformer uses this to deduce the current UTM fly zone
     * used for transforming stuff to latlon_origin.
     *
     * @param lat latitude in degrees
     * @param lon longitude in degrees
     */
    void setLatLon(const double lat, const double lon);

    //}

    /* transformSingle() //{ */

    /**
     * @brief Transforms a single message to a new frame.
     *
     * @param to_frame The target fram ID.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<T> transformSingle(const std::string& to_frame, const T& what);

    /**
     * @brief Transforms a single message to a new frame.
     *
     * A convenience override for shared pointers to const.
     *
     * @param to_frame The target fram ID.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transformSingle(const std::string& to_frame, const boost::shared_ptr<const T>& what)
    {
      auto ret = transformSingle(to_frame, *what);
      if (ret == std::nullopt)
        return std::nullopt;
      else
        return boost::make_shared<T>(std::move(ret.value()));
    }

    /**
     * @brief Transforms a single message to a new frame.
     *
     * A convenience override for shared pointers.
     *
     * @param to_frame The target fram ID.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transformSingle(const std::string& to_frame, const boost::shared_ptr<T>& what)
    {
      return transformSingle(to_frame, boost::shared_ptr<const T>(what));
    }


    //}

    /* transform() //{ */

    /**
     * @brief Transform a message to new frame using a particular transformation.
     *
     * @param tf The transformation to be used.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<T> transform(const geometry_msgs::TransformStamped& tf, const T& what);

    /**
     * @brief Transform a message to new frame using a particular transformation.
     *
     * A convenience override for shared pointers to const.
     *
     * @param tf The transformation to be used.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform(const geometry_msgs::TransformStamped& tf, const boost::shared_ptr<const T>& what)
    {
      auto ret = transform(tf, *what);
      if (ret == std::nullopt)
        return std::nullopt;
      else
        return boost::make_shared<T>(std::move(ret.value()));
    }

    /**
     * @brief Transform a message to new frame using a particular transformation.
     *
     * A convenience override for shared pointers.
     *
     * @param tf The transformation to be used.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform(const geometry_msgs::TransformStamped& tf, const boost::shared_ptr<T>& what)
    {
      return transform(tf, boost::shared_ptr<const T>(what));
    }

    //}

    /* getTransform() //{ */

    /**
     * @brief Gets a transform between two frames in a given time.
     *
     * @param from_frame The original frame of the transformation.
     * @param to_frame The target frame of the transformation.
     * @param time_stamp The time stamp of the transformation.
     *
     * @return \p std::nullopt if failed, optional containing the requested transformation otherwise.
     */
    [[nodiscard]] std::optional<geometry_msgs::TransformStamped> getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& time_stamp = ros::Time(0));

    //}

    /* resolveFrame() //{ */
    /**
     * @brief Deduce the full frame ID from a shortened or empty string.
     *
     * example:
     *   "" -> "uav1/gps_origin" (if the UAV is currently controlled in the gps_origin)
     *   "local_origin" -> "uav1/local_origin"
     *
     * @param frame_id The frame ID to be resolved.
     *
     * @return The resolved frame ID.
     */
    std::string resolveFrame(const std::string& frame_id);
    //}

    /* configuration methods //{ */
    void retryLookupNewest(const bool retry = true)
    {
      retry_lookup_newest_ = retry;
    }
    
    void beQuiet(const bool quiet = true)
    {
      quiet_ = quiet;
    }
    //}

  private:
    /* private members, methods etc //{ */

    /**
     * @brief keeps track whether a non-basic constructor was called and the transform listener is initialized
     */
    bool initialized_ = false;

    std::mutex mutex_tf_buffer_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

    std::string default_frame_id_;
    std::string prefix_;
    std::string node_name_;

    bool quiet_ = false;
    bool retry_lookup_newest_ = false;

    bool got_utm_zone_ = false;
    char utm_zone_[10] = {};

    std::string getFramePrefix(const std::string& frame_id);

    template <class T>
    std::optional<T> transformImpl(const geometry_msgs::TransformStamped& tf, const T& what);
    std::optional<mrs_msgs::ReferenceStamped> transformImpl(const geometry_msgs::TransformStamped& tf, const mrs_msgs::ReferenceStamped& what);
    std::optional<Eigen::Vector3d> transformImpl(const geometry_msgs::TransformStamped& tf, const Eigen::Vector3d& what);

    template <class T>
    std::optional<T> doTransform(const geometry_msgs::TransformStamped& tf, const T& what);

    //}

    // | ------------------- some helper methods ------------------ |
  public:
    /*  //{ */
    
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
    /*  //{ */
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

  public:
    Eigen::Vector3d LLtoUTM(const Eigen::Vector3d& what, [[maybe_unused]] const std::string& prefix);
    geometry_msgs::Point LLtoUTM(const geometry_msgs::Point& what, const std::string& prefix);
    geometry_msgs::Pose LLtoUTM(const geometry_msgs::Pose& what, const std::string& prefix);
    geometry_msgs::PoseStamped LLtoUTM(const geometry_msgs::PoseStamped& what, const std::string& prefix);

    std::optional<Eigen::Vector3d> UTMtoLL(const Eigen::Vector3d& what, [[maybe_unused]] const std::string& prefix);
    std::optional<geometry_msgs::Point> UTMtoLL(const geometry_msgs::Point& what, const std::string& prefix);
    std::optional<geometry_msgs::Pose> UTMtoLL(const geometry_msgs::Pose& what, const std::string& prefix);
    std::optional<geometry_msgs::PoseStamped> UTMtoLL(const geometry_msgs::PoseStamped& what, const std::string& prefix);
  };

#include <impl/transformer.hpp>

  //}

}  // namespace mrs_lib

#endif  // TRANSFORMER_H
