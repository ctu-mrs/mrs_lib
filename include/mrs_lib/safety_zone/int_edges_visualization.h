#ifndef INT_EDGES_VISUALIZATION_H
#define INT_EDGES_VISUALIZATION_H

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

// Visualizes the edges of the prism
// Horizontal edges are interactive and provide adding new vertex in the middle of the edge
// The visualization cannot be updated regularly, so lines disappear as camera moves "behind" them
// (view https://github.com/ros-visualization/rviz/issues/1287), therefore StaticEdgesVisualization
// class has been implemented.
class IntEdgesVisualization final : public Subscriber{
public:
  // Represents the prism
  IntEdgesVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh);

  // Represents corresponding obstacle in the safety_zone. 
  IntEdgesVisualization(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh);
  
  // Represents border of the safety_zone.
  IntEdgesVisualization(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh);

  ~IntEdgesVisualization();

  void update() override;
  void cleanup() override;

private:
  // Tools for convenience 
  void init();
  int getIndexByName(std::string marker_name);
  void addEdgeIntMarker(Point2d start, Point2d end, const double upper, const double lower, const int index);

  // Markers's callback
  void vertexAddCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // It is required for generating server id's, 
  // because their id's must be unique on topic 
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
  std::map<std::string, int> vertical_indecies_;
  std::map<int, std::string> upper_names_;
  std::map<int, std::string> lower_names_;
  std::map<int, std::string> vertical_names_;

}; // class IntEdgesVisualization
} // namespace mrs_lib


#endif