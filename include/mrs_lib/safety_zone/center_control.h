#ifndef CENTER_CONTROL_H
#define CENTER_CONTROL_H


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
  CenterControl(Prism* prism, std::string frame_id, ros::NodeHandle nh);
  void update();

private:
  void addIntMarker();
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
  void moveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void mouseUpCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // It is required for generating interactive marker names, 
  // because their names must be unique on topic 
  static int id_generator;

  const int id;

  Prism* prism_;
  std::string frame_id_;
  ros::NodeHandle nh_;

  interactive_markers::InteractiveMarkerServer* server_ = nullptr;
  std::string marker_name_;

  geometry_msgs::Quaternion last_orientation_;
  geometry_msgs::Point      last_position_;
  
}; // class CenterControl
} // namespace mrs_lib

#endif