#include "mrs_lib/safety_zone/bounds_control.h"

#include <tf/tf.h>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

namespace mrs_lib
{
int BoundsControl::id_generator = 0;

BoundsControl::BoundsControl(Prism& prism, std::string frame_id, ros::NodeHandle nh) : id_(id_generator), prism_(prism){
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

BoundsControl::BoundsControl(SafetyZone& safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh) : id_(id_generator), prism_(safety_zone.getObstacle(obstacle_id)) {
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

BoundsControl::BoundsControl(SafetyZone& safety_zone, std::string frame_id, ros::NodeHandle nh) : id_(id_generator), prism_(safety_zone.getBorder()){
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

void BoundsControl::init(){
  server_ = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "safety_area_bounds_out", std::to_string(id_), false);
  menu_handler_ = new interactive_markers::MenuHandler();
  menu_handler_->insert("Delete the prism", [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->deleteCallback(feedback);});

  prism_.subscribe(this);
  addBoundIntMarker(true);
  addBoundIntMarker(false);
}

void BoundsControl::update(){
  if(!prism_.isActive()){
    server_->clear();
    server_->applyChanges();
    return;
  }

  gm::Pose pose;
  pose.position.x = prism_.getCenter().get<0>();
  pose.position.y = prism_.getCenter().get<1>();
  pose.position.z = prism_.getMaxZ();

  server_->setPose(upper_name_, pose);
  pose.position.z = prism_.getMinZ();
  server_->setPose(lower_name_, pose);

  server_->applyChanges();
}

vm::Marker BoundsControl::makeBox(vm::InteractiveMarker &msg){
  vm::Marker marker;

  marker.type = vm::Marker::CUBE;
  marker.scale.x = msg.scale * 0.65;
  marker.scale.y = msg.scale * 0.65;
  marker.scale.z = msg.scale * 0.65;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

void BoundsControl::addBoundIntMarker(bool is_upper){
  Point2d center = prism_.getCenter();
  // Interactive marker
  vm::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.header.stamp.fromNSec(0);
  int_marker.pose.position.x = center.get<0>();
  int_marker.pose.position.y = center.get<1>();
  int_marker.scale = 1; 

  if(is_upper){
    int_marker.pose.position.z = prism_.getMaxZ();
    upper_name_ = std::to_string(id_) + "_upper";
    int_marker.name = upper_name_;
    int_marker.description = "Upper bound control";
  } else {
    int_marker.pose.position.z = prism_.getMinZ();
    lower_name_ = std::to_string(id_) + "_lower";
    int_marker.name = lower_name_;
    int_marker.description = "Lower bound control";
  }

  // Control
  vm::InteractiveMarkerControl control;
  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);

  // Z Moving element
  control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // XY Moving box
  control.interaction_mode = vm::InteractiveMarkerControl::MOVE_PLANE;
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // Send to server
  server_->insert(int_marker);
  menu_handler_->apply(*server_, int_marker.name);
  server_->setCallback(int_marker.name, 
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->boundMoveCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::POSE_UPDATE);
  server_->setCallback(int_marker.name, 
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->mouseDownCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::MOUSE_DOWN);
  server_->setCallback(int_marker.name, 
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->mouseUpCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::MOUSE_UP);
  server_->applyChanges();
  server_->applyChanges();
}

void BoundsControl::boundMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  double z = feedback->pose.position.z;
  if(feedback->marker_name == upper_name_){
    prism_.setMaxZ(z);
  }else if(feedback->marker_name == lower_name_){
    prism_.setMinZ(z);
  }else{
    ROS_WARN("[BoundsControl]: unknown marker appeared %s", feedback->marker_name.c_str());
  }

  if(!is_last_valid){
    return;
  }

  gm::Point current_position = feedback->pose.position;
  Point3d adjustment = Point3d{current_position.x - last_position_.x, 
                               current_position.y - last_position_.y,
                               0};
  prism_.move(adjustment);
  last_position_ = current_position;
}

void BoundsControl::mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  last_position_ = feedback->pose.position;
  is_last_valid = true;
}

void BoundsControl::mouseUpCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  is_last_valid = false;
}

void BoundsControl::deleteCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  prism_.deactivate();
}

}