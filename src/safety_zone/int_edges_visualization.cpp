#include "mrs_lib/safety_zone/int_edges_visualization.h"

#include "mrs_lib/attitude_converter.h"

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <geometry_msgs/Point.h>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

namespace mrs_lib {
int IntEdgesVisualization::id_generator = 0;

IntEdgesVisualization::IntEdgesVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh) : id_(id_generator), prism_(prism){
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

IntEdgesVisualization::IntEdgesVisualization(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh) : id_(id_generator), prism_(safety_zone->getObstacle(obstacle_id)) {
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

IntEdgesVisualization::IntEdgesVisualization(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh) : id_(id_generator), prism_(safety_zone->getBorder()) {
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

void IntEdgesVisualization::init() {
  server_ = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "safety_area_edges_out", std::to_string(id_), false);
  menu_handler_ = new interactive_markers::MenuHandler();
  menu_handler_->insert("Add vertex", [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->vertexAddCallback(feedback);});

  prism_->subscribe(this);

  auto polygon = prism_->getPolygon().outer();
  for(size_t i=0; i<polygon.size() - 1; i++){
    addEdgeIntMarker(polygon[i], polygon[i+1], prism_->getMaxZ(), prism_->getMinZ(), i);
  }
  update();
}

IntEdgesVisualization::~IntEdgesVisualization(){
  if(is_active_){
    prism_->unsubscribe(this);
  }
  cleanup();
  if(server_){
    delete server_;
  }
  if(menu_handler_){
    delete menu_handler_;
  }
}

void IntEdgesVisualization::update() {
  if(!is_active_){
    return;
  }
  
  auto polygon = prism_->getPolygon().outer();
  // Deleting extra vertices
  if(polygon.size() - 1 < upper_names_.size()){
    int initial_num = upper_names_.size();
    for(int i=polygon.size()-1; i<initial_num; i++){
      std::string upper_name = upper_names_[i];
      std::string lower_name = lower_names_[i];
      std::string vertical_name = vertical_names_[i];

      server_->erase(upper_name);
      server_->erase(lower_name);
      server_->erase(vertical_name);
      upper_names_.erase(i);
      lower_names_.erase(i);
      vertical_names_.erase(i);
      upper_indecies_.erase(upper_name);
      lower_indecies_.erase(lower_name);
      vertical_indecies_.erase(lower_name);
    }
  }

  // Adding not present vertices
  if(polygon.size() - 1 > upper_names_.size()){
    for(size_t i=upper_names_.size(); i<polygon.size() - 1; i++){
      addEdgeIntMarker(polygon[i], polygon[i+1], prism_->getMaxZ(), prism_->getMinZ(), i);
    }
  }
  
  // Updating the positions
  for(size_t i=0; i<polygon.size() - 1; i++){
    gm::Pose pose;
    pose.position.x = polygon[i].get<0>();
    pose.position.y = polygon[i].get<1>();
    pose.position.z = prism_->getMaxZ();

    gm::Point end;
    end.x = polygon[i+1].get<0>();
    end.y = polygon[i+1].get<1>();
    end.z = prism_->getMaxZ();

    vm::InteractiveMarker int_marker;

    // Upper marker    
    server_->setPose(upper_names_[i], pose);
    server_->get(upper_names_[i], int_marker);
    int_marker.controls[0].markers[0].points[0] = pose.position;
    int_marker.controls[0].markers[0].points[1] = end;
    server_->insert(int_marker);

    // Lower marker
    pose.position.z = prism_->getMinZ();
    end.z = prism_->getMinZ();
    server_->setPose(lower_names_[i], pose);
    server_->get(lower_names_[i], int_marker);
    int_marker.controls[0].markers[0].points[0] = pose.position;
    int_marker.controls[0].markers[0].points[1] = end;
    server_->insert(int_marker);

    // Vertical marker
    pose.position.z = prism_->getMaxZ();
    end.x = pose.position.x;
    end.y = pose.position.y;
    server_->setPose(vertical_names_[i], pose);
    server_->get(vertical_names_[i], int_marker);
    int_marker.controls[0].markers[0].points[0] = pose.position;
    int_marker.controls[0].markers[0].points[1] = end;
    server_->insert(int_marker);

    server_->applyChanges();
  }
}

void IntEdgesVisualization::cleanup() {
  server_->clear();
  server_->applyChanges();
  is_active_ = false;
}

int IntEdgesVisualization::getIndexByName(std::string marker_name){
  if(upper_indecies_.find(marker_name) != upper_indecies_.end() ){
    return upper_indecies_[marker_name];
  } else if(lower_indecies_.find(marker_name) != lower_indecies_.end()){
    return lower_indecies_[marker_name];
  } else{
    ROS_WARN("[VertexControl]: unknown marker appeared %s", marker_name.c_str());
    return -1;
  }
}

void IntEdgesVisualization::addEdgeIntMarker(Point2d start, Point2d end, const double upper, const double lower, const int index){
  // Visible line
  gm::Point p1;
  gm::Point p2;
  p1.x = start.get<0>();
  p1.y = start.get<1>();
  p1.z = upper;
  p2.x = end.get<0>();
  p2.y = end.get<1>();
  p2.z = upper;
  vm::Marker line;
  line.header.frame_id = frame_id_;
  line.type = vm::Marker::LINE_LIST;
  line.color.a = 0.3;
  line.color.r = 1.0;
  line.color.b = 0.0;
  line.color.g = 0.0;
  line.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
  line.scale.x = 0.2;

  // | ------------------ Upper bound -------------------- |
  // Interactive marker
  vm::InteractiveMarker upper_int_marker;
  upper_int_marker.header.frame_id = frame_id_;
  upper_int_marker.header.stamp.fromNSec(0);
  upper_int_marker.pose.position.x = start.get<0>();
  upper_int_marker.pose.position.y = start.get<1>();
  upper_int_marker.pose.position.z = upper;
  upper_int_marker.scale = 1; 
  upper_int_marker.name = std::to_string(id_) + "_upper_" + std::to_string(vertex_id_);// Each marker name must be unique

  // Control
  vm::InteractiveMarkerControl upper_control;
  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, upper_control.orientation);
  upper_control.interaction_mode = vm::InteractiveMarkerControl::MENU;
  upper_control.always_visible = true;

  // Add visible line
  line.points.clear();
  line.points.push_back(p1);
  line.points.push_back(p2);
  upper_control.markers.push_back(line);

  upper_int_marker.controls.push_back(upper_control);

  // Send to server_
  server_->insert(upper_int_marker);

  // Save
  upper_indecies_[upper_int_marker.name] = index;
  upper_names_[index] = upper_int_marker.name;

  // | ------------------ Lower bound -------------------- |
  // Interactive marker
  vm::InteractiveMarker lower_int_marker;
  lower_int_marker.header.frame_id = frame_id_;
  lower_int_marker.pose.position.x = start.get<0>();
  lower_int_marker.pose.position.y = start.get<1>();
  lower_int_marker.pose.position.z = lower;
  lower_int_marker.scale = 1; 
  lower_int_marker.name = std::to_string(id_) + "_lower_" + std::to_string(vertex_id_);// Each marker name must be unique

  // Control
  vm::InteractiveMarkerControl lower_control;
  tf::quaternionTFToMsg(orien, lower_control.orientation);
  lower_control.interaction_mode = vm::InteractiveMarkerControl::MENU;
  lower_control.always_visible = true;

  // Add visible line
  p1.z = lower;
  p2.z = lower;
  line.points.clear();
  line.points.push_back(p1);
  line.points.push_back(p2);
  lower_control.markers.push_back(line);

  lower_int_marker.controls.push_back(lower_control);

  // Send to server_
  server_->insert(lower_int_marker);

  // Save
  lower_indecies_[lower_int_marker.name] = index;
  lower_names_[index] = lower_int_marker.name;

  // | ---------------- Vertical edges ------------------- |
  // Interactive marker
  vm::InteractiveMarker vertical_int_marker;
  vertical_int_marker.header.frame_id = frame_id_;
  vertical_int_marker.pose.position.x = start.get<0>();
  vertical_int_marker.pose.position.y = start.get<1>();
  vertical_int_marker.pose.position.z = lower;
  vertical_int_marker.scale = 1; 
  vertical_int_marker.name = std::to_string(id_) + "_vertical_" + std::to_string(vertex_id_);// Each marker name must be unique

  // Control
  vm::InteractiveMarkerControl vertical_control;
  tf::quaternionTFToMsg(orien, vertical_control.orientation);
  vertical_control.interaction_mode = vm::InteractiveMarkerControl::NONE;
  vertical_control.always_visible = true;

  // Add visible line
  p1.x = start.get<0>();
  p1.y = start.get<1>();
  p1.z = upper;
  p2.x = p1.x;
  p2.y = p1.y;
  p2.z = lower;
  line.points.clear();
  line.points.push_back(p1);
  line.points.push_back(p2);
  vertical_control.markers.push_back(line);

  vertical_int_marker.controls.push_back(vertical_control);

  // Send to server_
  server_->insert(vertical_int_marker);

  // Save
  vertical_indecies_[vertical_int_marker.name] = index;
  vertical_names_[index] = vertical_int_marker.name;

  // | --------------------- Apply ----------------------- |
  vertex_id_ ++;
  menu_handler_->apply(*server_, upper_int_marker.name);
  menu_handler_->apply(*server_, lower_int_marker.name);
  server_->applyChanges();
}

void IntEdgesVisualization::vertexAddCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  int index = getIndexByName(feedback->marker_name);
  if(index < 0){
    return;
  }

  prism_->addVertexClockwise(index);
}

} // namespace mrs_lib