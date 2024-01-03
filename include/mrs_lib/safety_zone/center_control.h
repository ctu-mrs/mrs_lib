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
class CenterControl final : public Subscriber{
public:

  // Created interactive marker does not have deleting option
  CenterControl(Prism* prism, std::string frame_id, ros::NodeHandle nh);

  // Represents corresponding obstacle in the safety_zone. 
  // Created interactive marker provides deleting option
  CenterControl(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh);

  // Represents border of the safety_zone.
  // Created interactive marker does not have deleting option
  CenterControl(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh);

  ~CenterControl();

  void update() override;
  void cleanup() override;

private:
  // Tools for convenience 
  void init();
  void addIntMarker();
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);

  // Makrer's callbacks
  void moveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseUpCallback( [[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void deleteCallback( [[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // It is required for generating server id's, 
  // because their id's must be unique on topic 
  static int id_generator;
  const int id;

  // Attributes received in constructor
  Prism* prism_;
  SafetyZone* safety_zone_ = nullptr;
  const int obstacle_id_ = 0;
  std::string frame_id_;
  ros::NodeHandle nh_;

  // Communication with RVIZ
  interactive_markers::InteractiveMarkerServer* server_ = nullptr;
  interactive_markers::MenuHandler*             menu_handler_  = nullptr;
  std::string marker_name_;

  // Required for moving the prism
  geometry_msgs::Quaternion last_orientation_;
  geometry_msgs::Point      last_position_;
  bool                      is_last_valid = false;
  
}; // class CenterControl
} // namespace mrs_lib

#endif