#include "mrs_lib/safety_zone/prism_visualization.h"

#include "mrs_lib/attitude_converter.h"

#include <geometry_msgs/Point.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <tf/tf.h>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

namespace mrs_lib {
  int PrismVisualization::id_generator = 0;

  PrismVisualization::PrismVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh, double markers_update_rate) : id(id_generator) {
    prism_ = prism;
    frame_id_ = frame_id;
    nh_ = nh;
    static_markers_publisher_ = nh_.advertise<vm::Marker>("safety_area_static_markers_out", 1);
    static_markers_timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &PrismVisualization::updateStaticMarkers, this);
    id_generator++;

    // TODO: is it necessary to turn on separate thread? (the last argument in contructor)
    // TODO: add destructor
    vertex_server = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "safety_area_vertices_out", std::to_string(id), false);
    center_server = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "safety_area_center_out", std::to_string(id), false);
    bounds_server = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "safety_area_bounds_out", std::to_string(id), false);

    auto polygon = prism_->getPolygon().outer();
    for(int i=0; i<polygon.size() - 1; i++){
      addVertexIntMarker(polygon[i], prism_->getMaxZ(), prism->getMinZ(), i);
    }
  }

  // Lines must be updated periodically. View https://github.com/ros-visualization/rviz/issues/1287
  void PrismVisualization::updateStaticMarkers(const ros::TimerEvent& event) {
    if(!static_markers_update_required_){
      static_markers_publisher_.publish(static_marker_);
      return;
    }

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
    static_marker_ = marker;
    static_markers_publisher_.publish(static_marker_);
    static_markers_update_required_ = false;
  }

  vm::Marker PrismVisualization::makeBox(vm::InteractiveMarker &msg){
    vm::Marker marker;

    marker.type = vm::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
  }

  void PrismVisualization::addVertexIntMarker(mrs_lib::Point2d position, const double upper, const double lower, const int index){
    // | ------------------ Upper bound -------------------- |
    // Interactive marker
    vm::InteractiveMarker upper_int_marker;
    upper_int_marker.header.frame_id = frame_id_;
    upper_int_marker.header.stamp.fromNSec(0);
    upper_int_marker.pose.position.x = position.get<0>();
    upper_int_marker.pose.position.y = position.get<1>();
    upper_int_marker.pose.position.z = upper;
    upper_int_marker.scale = 1; 
    upper_int_marker.name = std::to_string(id) + "_upper_" + std::to_string(index);
    upper_int_marker.description = "Vertex control\n(2D Move)";

    // Control
    vm::InteractiveMarkerControl upper_control;
    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, upper_control.orientation);
    upper_control.interaction_mode = vm::InteractiveMarkerControl::MOVE_PLANE;

    // Moving element
    upper_int_marker.controls.push_back(upper_control);

    // Visible box
    upper_control.markers.push_back(makeBox(upper_int_marker));
    upper_control.always_visible = true;
    upper_int_marker.controls.push_back(upper_control);

    // Send to server
    vertex_server->insert(upper_int_marker);
    vertex_server->setCallback(upper_int_marker.name, 
        [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->vertexMoveCallback(feedback);}, 
        vm::InteractiveMarkerFeedback::POSE_UPDATE);

    // | ------------------ Lower bound -------------------- |
    // Interactive marker
    vm::InteractiveMarker lower_int_marker;
    lower_int_marker.header.frame_id = frame_id_;
    lower_int_marker.pose.position.x = position.get<0>();
    lower_int_marker.pose.position.y = position.get<1>();
    lower_int_marker.pose.position.z = lower;
    lower_int_marker.scale = 1; 
    lower_int_marker.name = std::to_string(id) + "_lower_" + std::to_string(index);
    lower_int_marker.description = "Vertex control\n(2D Move)";

    // Control
    vm::InteractiveMarkerControl lower_control;
    tf::quaternionTFToMsg(orien, lower_control.orientation);
    lower_control.interaction_mode = vm::InteractiveMarkerControl::MOVE_PLANE;

    // Moving element
    lower_int_marker.controls.push_back(lower_control);

    // Visible box
    lower_control.markers.push_back(makeBox(lower_int_marker));
    lower_control.always_visible = true;
    lower_int_marker.controls.push_back(lower_control);

    // Send to server
    vertex_server->insert(lower_int_marker);
    vertex_server->setCallback(lower_int_marker.name,
        [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->vertexMoveCallback(feedback);}, 
        vm::InteractiveMarkerFeedback::POSE_UPDATE);

    // | --------------------- Apply ----------------------- |
    vertex_server->applyChanges();
  }

  // Extracts index from string of form "[^_]*_[^_]*_[0-9]*"
  // Why: to get index from string "id_upper_index" or "id_lower_index" (those are the names of markers)
  int getIndexFromStr(const std::string& inputStr) {
    // Find the position of the first underscore
    size_t firstUnderscorePos = inputStr.find('_');
    size_t secondUnderscorePos = inputStr.find('_', firstUnderscorePos+1);
    
    // Extract the substring after the first underscore
    std::string indexStr = inputStr.substr(secondUnderscorePos + 1);
    
    // Convert the extracted substring to an integer
    int index = std::atoi(indexStr.c_str());

    return index;
  }

  // --------------------------------------------------------------
  // |                          Callbacks                         |
  // --------------------------------------------------------------

  void PrismVisualization::vertexMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    gm::Pose pose = feedback->pose;
    Point2d polygon_point = Point2d{pose.position.x, pose.position.y};
    int index = getIndexFromStr(feedback->marker_name);

    if(prism_->setVertex(polygon_point, index)){
      static_markers_update_required_ = true;

      // Current marker
      vertex_server->setPose(feedback->marker_name, pose);

      // Corresponding marker
      std::string name = feedback->marker_name;
      if(feedback->marker_name.find("upper") != std::string::npos){
        size_t index_to_replace = feedback->marker_name.find("upper");
        name.replace(index_to_replace, 5, "lower");
        pose.position.z = prism_->getMinZ();
      }
      else if(feedback->marker_name.find("lower") != std::string::npos){
        size_t index_to_replace = feedback->marker_name.find("lower");
        name.replace(index_to_replace, 5, "upper");
        pose.position.z = prism_->getMaxZ();
      } else{
        ROS_WARN("[PrismVisualization]: 'upper' or 'lower' is not present in the name of interactive marker. Current name = '%s'", feedback->marker_name.c_str());
        vertex_server->applyChanges();
        return;
      }
      vertex_server->setPose(name, pose);
      vertex_server->applyChanges();
    }
  }

} // namespace mrs_lib