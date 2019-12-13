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

/* defines //{ */

#define STRING_EQUAL 0

//}

namespace mrs_lib
{

typedef struct
{
  ros::Time                       stamp;
  geometry_msgs::TransformStamped tf;
} TransformCache_t;

/**
 * @brief tf wrapper that simplifies transforming stuff, adding caching, frame_id deduction and simple functions for one-time execution without getting the
 * transform first
 */
class Transformer {

public:
  /**
   * @brief the constructor inc. uav_name for namespacing and cache_timeout
   *
   * @param node_name the name of the node running the transformer, is used in ROS prints
   * @param uav_name the name of the UAV (a namespace), is used to deduce un-namespaced frame-ids
   * @param cache_timeout timeout in secs for the internal transform cache
   */
  Transformer(const std::string node_name, const std::string uav_name, const double cache_timeout);

  /**
   * @brief the constructor inc. cache_timeout
   *
   * @param node_name the name of the node running the transformer, is used in ROS prints
   * @param cache_timeout timeout in secs for the internal transform cache
   */
  Transformer(const std::string node_name, const double cache_timeout);

  /**
   * @brief the constructor inc. uav_name for namespacing, cache timeout defaults to 0.001 s
   *
   * @param node_name the name of the node running the transformer, is used in ROS prints
   * @param uav_name the name of the UAV (a namespace), is used to deduce un-namespaced frame-ids
   */
  Transformer(const std::string node_name, const std::string uav_name);

  /**
   * @brief the most basic constructor, cache timeout defaults to 0.001 s
   *
   * @param node_name the name of the node running the transformer, is used in ROS prints
   */
  Transformer(const std::string node_name);

  /**
   * @brief setter for the current frame_id into which the empty frame id will be deduced
   *
   * @param in the frame_id name
   */
  void setCurrentControlFrame(const std::string in);

  /**
   * @brief transform mrs_msgs::ReferenceStamped message to new frame
   *
   * @param to_frame target frame name
   * @param ref the reference to be transformed
   *
   * @return true if succeeded
   */
  bool transformReferenceSingle(const std::string to_frame, mrs_msgs::ReferenceStamped& ref);

  /**
   * @brief transform geometry_msgs::PoseStamped message to new frame
   *
   * @param to_frame target frame name
   * @param ref the reference to be transformed
   *
   * @return true if succeeded
   */
  bool transformPoseSingle(const std::string to_frame, geometry_msgs::PoseStamped& ref);

  /**
   * @brief transform geometry_msgs::Vector3Stamped message to new frame
   *
   * @param to_frame target frame name
   * @param ref the reference to be transformed
   *
   * @return true if succeeded
   */
  bool transformVector3Single(const std::string to_frame, geometry_msgs::Vector3Stamped& ref);

  /**
   * @brief transform mrs_msgs::ReferenceStamped message to new frame, given a particular tf
   *
   * @param tf the tf to be used
   * @param ref the reference to be transformed
   *
   * @return true if succeeded
   */
  bool transformReference(const geometry_msgs::TransformStamped& tf, mrs_msgs::ReferenceStamped& ref);

  /**
   * @brief transform geometry_msgs::PoseStamped message to new frame, given a particular tf
   *
   * @param tf the tf to be used
   * @param ref the reference to be transformed
   *
   * @return true if succeeded
   */
  bool transformPose(const geometry_msgs::TransformStamped& tf, geometry_msgs::PoseStamped& pose);

  /**
   * @brief transform geometry_msgs::Vector3Stamped message to new frame, given a particular tf
   *
   * @param tf the tf to be used
   * @param ref the reference to be transformed
   *
   * @return true if succeeded
   */
  bool transformVector3(const geometry_msgs::TransformStamped& tf, geometry_msgs::Vector3Stamped& vector3);

  /**
   * @brief gets a transform between two frames in a given time
   *
   * if it fails to find it for the particular time stamp, it uses the last available time
   *
   * @param from_frame the original frame
   * @param to_frame the target frame
   * @param time_stamp the time stamped to be used
   * @param tf reference to the transform
   *
   * @return true if succeeded
   */
  bool getTransform(const std::string from_frame, const std::string to_frame, const ros::Time time_stamp, geometry_msgs::TransformStamped& tf);

private:
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::mutex                                  mutex_tf_buffer_;

  std::string node_name_;

  std::string uav_name_;
  bool        got_uav_name_ = false;

  double cache_timeout_;

  std::string current_control_frame_;
  bool        got_current_control_frame_ = false;
  std::mutex  mutex_current_control_frame_;

  std::string resolveFrameName(const std::string in);

  // caches the last calculated transforms
  // if someone asks for tf which was calculated within the last 1 ms
  // it is pulled from a cache
  std::map<std::string, TransformCache_t> transformer_cache_;
  std::mutex                              mutex_transformer_cache_;
};

}  // namespace mrs_lib

#endif  // TRANSFORMER_H
