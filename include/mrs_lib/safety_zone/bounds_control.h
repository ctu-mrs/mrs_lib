#ifndef BOUNDS_CONTROL_H
#define BOUNDS_CONTROL_H

#include "prism.h"
#include "safety_zone.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <ros/ros.h>

namespace mrs_lib
{

//  Creates 2 interactive markers in the center of the prism's polygon:
// one for maximum height of the prism and the other for minimum height.
// Both markers can be used for moving entire prism and deleting it (view constructors' docstring)
// Markers also allow changing correspondings heights
class BoundsControl : public Subscriber{
public:

  // Created interactive markers do not have deleting option
  BoundsControl(Prism* prism, std::string frame_id, ros::NodeHandle nh);

  // Represents corresponding obstacle in the safety_zone. 
  // Created interactive markers provide deleting option
  BoundsControl(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh);

  // Represents border of the safety_zone.
  // Created interactive markers do not have deleting option
  BoundsControl(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh);

  ~BoundsControl();

  void update();
  void cleanup();

private:
  void init();
  void addBoundIntMarker(bool is_upper);
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
  void boundMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseUpCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void deleteCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // It is required for generating server id's, 
  // because their id's must be unique on topic 
  static int id_generator;
  const int id_;
  const int obstacle_id_ = 0;

  Prism* prism_;
  SafetyZone* safety_zone_ = nullptr;
  std::string frame_id_;
  ros::NodeHandle nh_;

  interactive_markers::InteractiveMarkerServer* server_ = nullptr;
  interactive_markers::MenuHandler*             menu_handler_ = nullptr;
  std::string upper_name_;
  std::string lower_name_;
  
  geometry_msgs::Point      last_position_;
  bool                      is_last_valid = false;
}; // class BoundsControl
} // namespace mrs_lib

#endif