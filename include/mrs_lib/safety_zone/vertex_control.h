#ifndef VERTEX_CONTROL_H
#define VERTEX_CONTROL_H

#include "prism.h"
#include "safety_zone.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <ros/ros.h>
#include <map>

namespace mrs_lib 
{

//  Creates 2 interactive markers for eache vertex:
// one for maximum height of the prism and the other for minimum height.
// The markers can be used to change X and Y coordinates of corresponding vertex.
// Provides the option to delete the corresponding vertex
class VertexControl final : public Subscriber{

public:
  // Represents the prism
  VertexControl(Prism* prism, std::string frame_id, ros::NodeHandle nh);

  // Represents corresponding obstacle in the safety_zone. 
  VertexControl(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh);
  
  // Represents border of the safety_zone.
  VertexControl(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh);

  ~VertexControl();

  void update() override;
  void cleanup() override;

private:
  // Tools for convenience 
  void init();
  int getIndexByName(std::string marker_name);
  void addVertexIntMarker(Point2d position, const double upper, const double lower, const int index);
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);

  // Markers' callbacks
  void vertexMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void vertexDeleteCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // It is required for generating interactive marker names, 
  // because their names must be unique on topic 
  static int id_generator;
  const int id_;
  int vertex_id_ = 0;    // Each marker name must be unique
  
  // Attributes received in constructor
  Prism* prism_;
  std::string frame_id_;
  ros::NodeHandle nh_;

  // Interactive markers
  interactive_markers::InteractiveMarkerServer* server_ = nullptr;
  interactive_markers::MenuHandler*             menu_handler_  = nullptr;

  // Note:
  // Since no method find_by_value() is present and markers may not be ordered,
  // it was decided to make several containers.
  // std::map takes O(n) amount of memory and CRUD operations have logarithmic complexity.
  // Because there will rarely more than 10 vertecies per prism and vertexMoveCallback can
  // be called more than 10 times per second, such containers fit perfectly 
  std::map<std::string, int> upper_indecies_;
  std::map<std::string, int> lower_indecies_;
  std::map<int, std::string> upper_names_;
  std::map<int, std::string> lower_names_;

}; // class VertexControl
} // namespace mrs_lib

#endif