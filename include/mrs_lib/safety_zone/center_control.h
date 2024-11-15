#ifndef CENTER_CONTROL_H
#define CENTER_CONTROL_H

#include "safety_zone.h"
#include "prism.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <ros/ros.h>

namespace mrs_lib
{


  //  Creates interactive marker
  // XY coordinates: center of the polygon of the prism
  // Z coordinate: center of [MinZ; MaxZ] interval
  // Marker provides moving entire prism along XY and Z axis, rotating it and
  // deleting it (view constructors' docstring)
  // Markers also allow changing correspondings heights
  class CenterControl final : public Subscriber
  {
  public:
    // Created interactive marker does not have deleting option
    CenterControl(Prism* prism, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh);
    // Represents corresponding obstacle in the safety_zone.
    // Created interactive marker provides deleting option
    CenterControl(SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh);
    // Represents border of the safety_zone.
    // Created interactive marker does not have deleting option
    CenterControl(SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh);

    ~CenterControl();

    void update() override;
    void cleanup() override;

  private:
    // Tools for convenience
    void init();
    void addIntMarker();
    visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker& msg);

    // Marker's callbacks
    void moveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void mouseUpCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void deleteCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // It is required for generating server id's,
    // because their id's must be unique on topic
    static std::atomic<int> id_generator_;
    const int id_;

    // Attributes received in constructor
    Prism* prism_;
    const int obstacle_id_ = 0;
    std::string uav_name_;
    std::string frame_id_;
    ros::NodeHandle nh_;
    SafetyZone* safety_zone_ = nullptr;

    // Communication with RVIZ
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_ = nullptr;
    std::unique_ptr<interactive_markers::MenuHandler> menu_handler_ = nullptr;
    std::string marker_name_;

    // Required for moving the prism
    geometry_msgs::Quaternion last_orientation_;
    geometry_msgs::Point last_position_;
    bool is_last_valid_ = false;

  };  // class CenterControl
}  // namespace mrs_lib

#endif
