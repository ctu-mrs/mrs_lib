#include "mrs_lib/safety_zone/edges_visualization.h"

#include "mrs_lib/attitude_converter.h"

#include <geometry_msgs/Point.h>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

namespace mrs_lib {

EdgesVisualization::EdgesVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh, double markers_update_rate) {
  prism_ = prism;
  frame_id_ = frame_id;
  nh_ = nh;
  publisher_ = nh_.advertise<vm::Marker>("safety_area_static_markers_out", 1);
  timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &EdgesVisualization::sendMarker, this);
  prism_->subscribe(this);
  update();
}

// Lines must be updated periodically. View https://github.com/ros-visualization/rviz/issues/1287
void EdgesVisualization::sendMarker(const ros::TimerEvent& event) {
  publisher_.publish(last_marker_);
}

void EdgesVisualization::update() {
  double max_z = prism_->getMaxZ();
  double min_z = prism_->getMinZ();
  auto polygon = prism_->getPolygon().outer();
  
  vm::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.type = vm::Marker::LINE_LIST;
  marker.color.a = 0.3;
  marker.color.r = 1.0;
  marker.color.b = 0.0;
  marker.color.g = 0.0;
  marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
  marker.scale.x = 0.2;

  for(int i=0; i<polygon.size() - 1; i++){  // -1 because the last is the same as the first one      
    // Adding upper horizontal edges
    gm::Point upper_start;
    gm::Point upper_end;

    upper_start.x = polygon[i].get<0>();
    upper_start.y = polygon[i].get<1>();
    upper_start.z = max_z;

    upper_end.x = polygon[i+1].get<0>();
    upper_end.y = polygon[i+1].get<1>();
    upper_end.z = max_z;

    marker.points.push_back(upper_start);
    marker.points.push_back(upper_end);

    // Adding lower horizontal edges
    gm::Point lower_start;
    gm::Point lower_end;

    lower_start.x = polygon[i].get<0>();
    lower_start.y = polygon[i].get<1>();
    lower_start.z = min_z;

    lower_end.x = polygon[i+1].get<0>();
    lower_end.y = polygon[i+1].get<1>();
    lower_end.z = min_z;

    marker.points.push_back(lower_start);
    marker.points.push_back(lower_end);

    // Adding vertical edges
    marker.points.push_back(upper_start);
    marker.points.push_back(lower_start);
  }
  last_marker_ = marker;
  publisher_.publish(last_marker_);
}

} // namespace mrs_lib