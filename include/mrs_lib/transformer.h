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

#include <mrs_lib/attitude_converter.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>

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

    /* set_default_frame() //{ */

    /**
     * @brief Sets the default frame ID to be used instead of any empty frame ID.
     *
     * @param The default frame ID.
     */
    void set_default_frame(const std::string& frame_id)
    {
      default_frame_id_ = frame_id;
    }

    //}

    /* set_default_prefix() //{ */

    /**
     * @brief Sets the default frame ID prefix to be used if no prefix is present in the frame.
     *
     * @param The default frame ID prefix.
     */
    void set_default_prefix(const std::string& prefix)
    {
      prefix_ = prefix;
    }

    //}

    /* set_lat_lon() //{ */

    /**
     * @brief Sets the curret lattitude and longitude for UTM zone calculation.
     *
     * The transformer uses this to deduce the current UTM fly zone
     * used for transforming stuff to latlon_origin.
     *
     * @param lat latitude in degrees
     * @param lon longitude in degrees
     */
    void set_lat_lon(const double lat, const double lon);

    //}

    /* transform_single() //{ */

    /**
     * @brief Transforms a single message to a new frame.
     *
     * @param to_frame The target fram ID.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<T> transform_single(const std::string& to_frame, const T& what);

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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform_single(const std::string& to_frame, const boost::shared_ptr<const T>& what)
    {
      auto ret = transform_single(to_frame, *what);
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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform_single(const std::string& to_frame, const boost::shared_ptr<T>& what)
    {
      return transform_single(to_frame, boost::shared_ptr<const T>(what));
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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform(const mrs_lib::TransformStamped& tf, const boost::shared_ptr<const T>& what)
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
    [[nodiscard]] std::optional<boost::shared_ptr<T>> transform(const mrs_lib::TransformStamped& tf, const boost::shared_ptr<T>& what)
    {
      return transform(tf, boost::shared_ptr<const T>(what));
    }

    /**
     * @brief Transform a message that does not contain a header to new frame using a particular transformation.
     *
     * A convenience override for messages without a header (need a little special treatment).
     *
     * @param tf The transformation to be used.
     * @param what The object to be transformed.
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise.
     */
    template <class T>
    [[nodiscard]] std::optional<T> transform_headerless(const mrs_lib::TransformStamped& tf, const T& what);

    //}

    /* get_transform() //{ */

    /**
     * @brief Gets a transform between two frames in a given time.
     *
     * @param from_frame The original frame of the transformation.
     * @param to_frame The target frame of the transformation.
     * @param time_stamp The time stamp of the transformation.
     * @param use_last_if_fail If true, the latest available transformation will be used instead of time_stamp if the lookup fails.
     * @param quiet Should not print excessively.
     *
     * @return \p std::nullopt if failed, optional containing the requested transformation otherwise.
     */
    [[nodiscard]] std::optional<mrs_lib::TransformStamped> get_transform(const std::string& from_frame, const std::string& to_frame,
                                                                         const ros::Time& time_stamp = ros::Time(0), const bool use_last_if_fail = false, const bool quiet = false);

    //}

    /* resolve_frame() //{ */
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
    std::string resolve_frame(const std::string& frame_id);
    //}

  private:
    /* private members, methods etc //{ */

    std::string default_frame_id_;

    std::mutex mutex_tf_buffer_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

    std::string node_name_;

    std::string prefix_;

    std::mutex mutex_current_control_frame_;
    std::string current_control_frame_;
    bool got_current_control_frame_ = false;

    std::mutex mutex_utm_zone_;
    bool got_utm_zone_ = false;
    char utm_zone_[10];

    std::string getUAVFramePrefix(const std::string& in);

    template <class T>
    std::optional<T> transformImpl(const mrs_lib::TransformStamped& tf, const T& what);
    std::optional<mrs_msgs::ReferenceStamped> transformImpl(const mrs_lib::TransformStamped& tf, const mrs_msgs::ReferenceStamped& what);
    std::optional<geometry_msgs::PoseStamped> transformImpl(const mrs_lib::TransformStamped& tf, const geometry_msgs::PoseStamped& what);
    std::optional<geometry_msgs::Point> transformImpl(const mrs_lib::TransformStamped& tf, const geometry_msgs::Point& what);
    std::optional<Eigen::Vector3d> transformImpl(const mrs_lib::TransformStamped& tf, const Eigen::Vector3d& what);
    /* std::optional<Eigen::MatrixXd> transformImpl(const mrs_lib::TransformStamped& tf, const Eigen::MatrixXd& what); */
    /* std::optional<Eigen::MatrixXd> transformMat2(const mrs_lib::TransformStamped& tf, const Eigen::MatrixXd& what); */
    /* std::optional<Eigen::MatrixXd> transformMat3(const mrs_lib::TransformStamped& tf, const Eigen::MatrixXd& what); */

    template <class T>
    std::optional<T> doTransform(const mrs_lib::TransformStamped& tf, const T& what);

    geometry_msgs::PoseStamped prepareMessage(const mrs_msgs::ReferenceStamped& what);
    mrs_msgs::ReferenceStamped postprocessMessage(const geometry_msgs::PoseStamped& what);

    /**
     * @brief keeps track whether a non-basic constructor was launched and the transform listener was initialized
     */
    bool initialized_ = false;

    //}

  };

#include <impl/transformer.hpp>

  //}

}  // namespace mrs_lib

#endif  // TRANSFORMER_H
