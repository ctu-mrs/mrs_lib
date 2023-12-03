#ifndef MRS_PRISM_VISUALIZATION_H
#define MRS_PRISM_VISUALIZATION_H

#include "prism.h"

#include "mrs_lib/publisher_handler.h"
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <ros/ros.h>

namespace mrs_lib 
{
class PrismVisualization{

public:
  PrismVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh, double markers_update_rate);
  ~PrismVisualization() {}

private:
  void updateStaticMarkers(const ros::TimerEvent& event);
  void addVertexIntMarker(mrs_lib::Point2d position, const double upper, const double lower, const int index);
  void addCenterIntMarker(mrs_lib::Point3d position);
  void addBoundsIntMarker(mrs_lib::Point2d position, const double upper, const double lower);
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);

  // Interactive markers callbacks
  void vertexMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // Is required for generating interactive marker names, 
  // because their names must be unique on topic 
  static int id_generator;

  const int id;
  Prism* prism_;
  std::string frame_id_;
  ros::NodeHandle nh_;

  // Static markers
  ros::Publisher static_markers_publisher_;
  ros::Timer static_markers_timer_;
  visualization_msgs::Marker static_marker_;
  bool static_markers_update_required_ = true;

  // Interactive markers
  interactive_markers::InteractiveMarkerServer* vertex_server = nullptr;
  interactive_markers::InteractiveMarkerServer* center_server = nullptr;
  interactive_markers::InteractiveMarkerServer* bounds_server = nullptr;
  interactive_markers::MenuHandler*             menu_handler  = nullptr;

}; // class PrismVisualization
} // namespace mrs_lib

#endif