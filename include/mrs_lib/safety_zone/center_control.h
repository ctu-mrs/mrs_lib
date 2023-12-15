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
class CenterControl : public Subscriber{
public:
  CenterControl(Prism& prism, std::string frame_id, ros::NodeHandle nh);
  CenterControl(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh);
  CenterControl(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh);

  ~CenterControl();

  void update();
  void cleanup();

private:
  void init();
  void addIntMarker();
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
  void moveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseUpCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void deleteCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // It is required for generating interactive marker names, 
  // because their names must be unique on topic 
  static int id_generator;
  const int id;
  const int obstacle_id_ = 0;

  Prism& prism_;
  SafetyZone* safety_zone_ = nullptr;
  std::string frame_id_;
  ros::NodeHandle nh_;

  interactive_markers::InteractiveMarkerServer* server_ = nullptr;
  interactive_markers::MenuHandler*             menu_handler_  = nullptr;
  std::string marker_name_;

  geometry_msgs::Quaternion last_orientation_;
  geometry_msgs::Point      last_position_;
  bool                      is_last_valid = false;
  
}; // class CenterControl
} // namespace mrs_lib

#endif