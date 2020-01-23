// clang: MatousFormat

/**  \file
 *   \brief A wrapper for easier work with tf2 transformations.
 *   \author Tomas Baca - tomas.baca@fel.cvut.cz
 */
#ifndef TRANSFORMER_H
#define TRANSFORMER_H

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

#include <mutex>

//}

namespace mrs_lib
{

  static const std::string UTM_ORIGIN = "utm_origin";
  static const std::string LATLON_ORIGIN = "latlon_origin";

  /**
   * @brief a wrapper around the standard transform object
   */
  /* class TransformStamped //{ */

  class TransformStamped
  {

  public:
    TransformStamped();
    TransformStamped(const std::string from_frame, const std::string to_frame, const ros::Time stamp);
    TransformStamped(const std::string from_frame, const std::string to_frame, const ros::Time stamp, const geometry_msgs::TransformStamped transform_stamped);

    std::string from(void) const;
    std::string to(void) const;
    ros::Time stamp(void) const;
    geometry_msgs::TransformStamped getTransform(void) const;

  private:
    geometry_msgs::TransformStamped transform_stamped_;

    std::string from_frame_;
    std::string to_frame_;
    ros::Time stamp_;
  };

  //}

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
     * @brief the constructor inc. uav_name for namespacing
     *
     * @param node_name the name of the node running the transformer, is used in ROS prints
     * @param uav_name the name of the UAV (a namespace), is used to deduce un-namespaced frame-ids
     */
    Transformer(const std::string& node_name, const std::string& uav_name);

    /**
     * @brief the constructor
     *
     * @param node_name the name of the node running the transformer, is used in ROS prints
     */
    Transformer(const std::string& node_name);

    /**
     * @brief the copy constructor
     *
     * @param other other object
     */
    Transformer(const Transformer& other);

    //}

    /* Assignment operator //{ */

    /**
     * @brief the assignment operator
     *
     * @param other other object
     *
     * @return this
     */
    Transformer& operator=(const Transformer& other);

    //}

    /* Destructor //{ */

    /**
     * @brief the basic destructor
     */
    ~Transformer();

    //}

    /* setCurrentControlFrame() //{ */

    /**
     * @brief setter for the current frame_id into which the empty frame id will be deduced
     *
     * @param in the frame_id name
     */
    void setCurrentControlFrame(const std::string& in);

    //}

    /* setCurrentLatLon() //{ */

    /**
     * @brief set the curret lattitu and longitu
     *
     * The transformer uses this to deduce the current fly zone,
     * so it could transform stuff back to latlon_origin.
     *
     * @param lat latitude in degrees
     * @param lon longitude in degrees
     */
    void setCurrentLatLon(const double lat, const double lon);

    //}

    /* transformSingle() //{ */

    /**
     * @brief transform a message to new frame
     *
     * @param to_frame target frame name
     * @param what the object to be transformed
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise
     */
    template <class T>
    std::optional<T> transformSingle(const std::string& to_frame, const T& what);

    //}

    /* transform() //{ */

    /**
     * @brief transform a message to new frame, given a particular tf
     *
     * @param tf the tf to be used
     * @param what the object to be transformed
     *
     * @return \p std::nullopt if failed, optional containing the transformed object otherwise
     */
    template <class T>
    std::optional<T> transform(const mrs_lib::TransformStamped& tf, const T& what);

    //}

    /* getTransform() //{ */

    /**
     * @brief gets a transform between two frames in a given time
     *
     * if it fails to find it for the particular time stamp, it uses the last available time
     *
     * @param from_frame the original frame
     * @param to_frame the target frame
     * @param time_stamp the time stamped to be used
     *
     * @return \p std::nullopt if failed, optional containing the requested transform otherwise
     */
    std::optional<mrs_lib::TransformStamped> getTransform(const std::string& from_frame, const std::string& to_frame, const ros::Time& time_stamp);

    //}

  private:
    /* private members, methods etc //{ */

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
    std::mutex mutex_tf_buffer_;

    std::string node_name_;

    std::string uav_name_;
    bool got_uav_name_ = false;

    std::string current_control_frame_;
    bool got_current_control_frame_ = false;
    mutable std::mutex mutex_current_control_frame_;

    bool got_utm_zone_ = false;
    char utm_zone_[10];
    std::mutex mutex_utm_zone_;

    /**
     * @brief deduced the full frame_id from a shortened or empty string
     *
     * example:
     *   "" -> "uav1/gps_origin" (if the UAV is currently controlled in the gps_origin)
     *   "local_origin" -> "uav1/local_origin"
     *
     * @param in the frame_id to be resolved
     *
     * @return resolved frame_id
     */
    std::string resolveFrameName(const std::string& in);

    std::string getUAVFramePrefix(const std::string& in);

    template <class T>
    std::optional<T> transformImpl(const mrs_lib::TransformStamped& tf, const T& what);
    std::optional<mrs_msgs::ReferenceStamped> transformImpl(const mrs_lib::TransformStamped& tf, const mrs_msgs::ReferenceStamped& what);
    std::optional<geometry_msgs::PoseStamped> transformImpl(const mrs_lib::TransformStamped& tf, const geometry_msgs::PoseStamped& what);

    template <class T>
    std::optional<T> doTransform(const mrs_lib::TransformStamped& tf, const T& what);

    geometry_msgs::PoseStamped prepareMessage(const mrs_msgs::ReferenceStamped& what);
    mrs_msgs::ReferenceStamped postprocessMessage(const geometry_msgs::PoseStamped& what);

    /**
     * @brief keeps track whether a non-basic constructor was launched and the transform listener was initialized
     */
    bool is_initialized_ = false;

    //}
  };

#include <impl/transformer.hpp>

  //}

}  // namespace mrs_lib

#endif  // TRANSFORMER_H
