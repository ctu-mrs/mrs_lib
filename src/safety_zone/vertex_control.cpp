#include "mrs_lib/safety_zone/vertex_control.h"

#include "mrs_lib/attitude_converter.h"

#include <geometry_msgs/Point.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <tf/tf.h>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

namespace mrs_lib {
int VertexControl::id_generator = 0;

VertexControl::VertexControl(Prism* prism, std::string frame_id, ros::NodeHandle nh) : id_(id_generator) {
  prism_ = prism;
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;

  server_ = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "safety_area_vertices_out", std::to_string(id_), false);

  auto polygon = prism_->getPolygon().outer();
  for(int i=0; i<polygon.size() - 1; i++){
    addVertexIntMarker(polygon[i], prism_->getMaxZ(), prism->getMinZ(), i);
  }

  prism_->subscribe(this);
}

VertexControl::~VertexControl(){
  if(server_){
    delete server_;
  }
  if(menu_handler_){
    delete menu_handler_;
  }
}

void VertexControl::update(){
  auto polygon = prism_->getPolygon().outer();

  if(upper_names_.size() != polygon.size() -1 || lower_names_.size() != polygon.size() - 1){
    ROS_WARN("[VertexControl]: Could not update markers, because number of markers and number of verticies are different");
    return;
  }

  for(int i=0; i<polygon.size() - 1; i++){
    gm::Pose pose;
    pose.position.x = polygon[i].get<0>();
    pose.position.y = polygon[i].get<1>();
    pose.position.z = prism_->getMaxZ();

    server_->setPose(upper_names_[i], pose);
    pose.position.z = prism_->getMinZ();
    server_->setPose(lower_names_[i], pose);

    server_->applyChanges();
  }
}

vm::Marker VertexControl::makeBox(vm::InteractiveMarker &msg){
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

void VertexControl::addVertexIntMarker(mrs_lib::Point2d position, const double upper, const double lower, const int index){
  // | ------------------ Upper bound -------------------- |
  // Interactive marker
  vm::InteractiveMarker upper_int_marker;
  upper_int_marker.header.frame_id = frame_id_;
  upper_int_marker.header.stamp.fromNSec(0);
  upper_int_marker.pose.position.x = position.get<0>();
  upper_int_marker.pose.position.y = position.get<1>();
  upper_int_marker.pose.position.z = upper;
  upper_int_marker.scale = 1; 
  upper_int_marker.name = std::to_string(id_) + "_upper_" + std::to_string(vertex_id_);// Each marker name must be unique
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

  // Send to server_
  server_->insert(upper_int_marker);
  server_->setCallback(upper_int_marker.name, 
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->vertexMoveCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::POSE_UPDATE);

  // Save
  upper_indecies_[upper_int_marker.name] = index;
  upper_names_[index] = upper_int_marker.name;

  // | ------------------ Lower bound -------------------- |
  // Interactive marker
  vm::InteractiveMarker lower_int_marker;
  lower_int_marker.header.frame_id = frame_id_;
  lower_int_marker.pose.position.x = position.get<0>();
  lower_int_marker.pose.position.y = position.get<1>();
  lower_int_marker.pose.position.z = lower;
  lower_int_marker.scale = 1; 
  lower_int_marker.name = std::to_string(id_) + "_lower_" + std::to_string(vertex_id_);// Each marker name must be unique
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

  // Send to server_
  server_->insert(lower_int_marker);
  server_->setCallback(lower_int_marker.name,
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->vertexMoveCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::POSE_UPDATE);

  // Save
  lower_indecies_[lower_int_marker.name] = index;
  lower_names_[index] = lower_int_marker.name;

  // | --------------------- Apply ----------------------- |
  vertex_id_ ++;
  server_->applyChanges();
}

// --------------------------------------------------------------
// |                          Callbacks                         |
// --------------------------------------------------------------

void VertexControl::vertexMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  gm::Pose pose = feedback->pose;
  Point2d polygon_point = Point2d{pose.position.x, pose.position.y};
  int index;

  if(upper_indecies_.find(feedback->marker_name) != upper_indecies_.end()){
    index = upper_indecies_[feedback->marker_name];
  } else if (lower_indecies_.find(feedback->marker_name) != lower_indecies_.end()){
    index = lower_indecies_[feedback->marker_name];
  } else{
    ROS_WARN("[VertexControl]: unknown marker appeared %s", feedback->marker_name.c_str());
    return;
  }

  prism_->setVertex(polygon_point, index);
}

} // namespace mrs_lib