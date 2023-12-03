#ifndef VERTEX_CONTROL_H
#define VERTEX_CONTROL_H

#include "prism.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <ros/ros.h>
#include <map>

namespace mrs_lib 
{
class VertexControl : public Subscriber{

public:
  VertexControl(Prism* prism, std::string frame_id, ros::NodeHandle nh);

  ~VertexControl();

  void update();

private:
  void addVertexIntMarker(Point2d position, const double upper, const double lower, const int index);
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
  void vertexMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // It is required for generating interactive marker names, 
  // because their names must be unique on topic 
  static int id_generator;

  const int id;
  int vertex_id = 0;    // Each marker name must be unique
  Prism* prism_;
  std::string frame_id_;
  ros::NodeHandle nh_;

  // Interactive markers
  interactive_markers::InteractiveMarkerServer* server = nullptr;
  interactive_markers::MenuHandler*             menu_handler  = nullptr;

  // Note:
  // Since no method find_by_value() is present and markers may not be ordered,
  // it was decided to make several containers.
  // std::map takes O(n) amount of memory and CRUD operations have logarithmic complexity.
  // Because there will rarely more than 10 vertecies per prism and vertexMoveCallback may
  // be called more than 10 times per second, such containers fit perfectly 
  std::map<std::string, int> upper_indecies;
  std::map<std::string, int> lower_indecies;
  std::map<int, std::string> upper_names;
  std::map<int, std::string> lower_names;

}; // class VertexControl
} // namespace mrs_lib

#endif