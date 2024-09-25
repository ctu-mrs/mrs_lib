#include "mrs_lib/safety_zone/center_control.h"

#include <tf/tf.h>
#include <numeric>
#include <math.h>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;
namespace bg = boost::geometry;

namespace mrs_lib
{
int CenterControl::id_generator = 0;

CenterControl::CenterControl(Prism* prism, std::string frame_id, ros::NodeHandle nh) : id(id_generator), prism_(prism){
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

CenterControl::CenterControl(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh) : id(id_generator), obstacle_id_(obstacle_id), prism_(safety_zone->getObstacle(obstacle_id)) {
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  safety_zone_ = safety_zone;
  init();
  std::cout << "center vis of obstacle inited\n";
}

CenterControl::CenterControl(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh) : id(id_generator), prism_(safety_zone->getBorder()){
  frame_id_ = frame_id;
  nh_ = nh;
  id_generator++;
  init();
}

CenterControl::~CenterControl(){
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


void CenterControl::init(){
  server_ = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "/safety_area_center_out", std::to_string(id), false);
  menu_handler_ = new interactive_markers::MenuHandler();
  if(safety_zone_){
    menu_handler_->insert("Delete the prism", [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->deleteCallback(feedback);});
  }else{
    menu_handler_->insert("Deleting is disabled for this prism");
  }

  prism_->subscribe(this);
  addIntMarker();
}

void CenterControl::update(){
  if(!is_active_){
    return;
  }

  gm::Pose pose;
  pose.position.x = prism_->getCenter().get<0>();
  pose.position.y = prism_->getCenter().get<1>();
  pose.position.z = (prism_->getMaxZ() + prism_->getMinZ()) / 2;

  server_->setPose(marker_name_, pose);
  server_->applyChanges();
}

void CenterControl::cleanup(){
  std::cout << "center cleanup called " << id << std::endl;
  server_->clear();
  server_->applyChanges();
  is_active_ = false;
}

vm::Marker CenterControl::makeBox(vm::InteractiveMarker &msg){
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

void CenterControl::addIntMarker(){
  Point2d center = prism_->getCenter();
  // Interactive marker
  vm::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.header.stamp.fromNSec(0);
  int_marker.pose.position.x = center.get<0>();
  int_marker.pose.position.y = center.get<1>();
  int_marker.pose.position.z = (prism_->getMaxZ() + prism_->getMinZ()) / 2;
  int_marker.scale = 3; 
  int_marker.name = std::to_string(id);

  // Control
  vm::InteractiveMarkerControl control;
  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);

  // Rotating around z-axes
  control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Along z axes
  control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Along x-y axes (box)
  control.interaction_mode = vm::InteractiveMarkerControl::MOVE_PLANE;
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // Send to server
  server_->insert(int_marker);
  menu_handler_->apply(*server_, int_marker.name);
  server_->setCallback(int_marker.name, 
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->moveCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::POSE_UPDATE);
  server_->setCallback(int_marker.name, 
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->mouseDownCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::MOUSE_DOWN);
  server_->setCallback(int_marker.name, 
      [this](const vm::InteractiveMarkerFeedbackConstPtr &feedback){this->mouseUpCallback(feedback);}, 
      vm::InteractiveMarkerFeedback::MOUSE_UP);
  server_->applyChanges();
  marker_name_ = int_marker.name;
}

void CenterControl::moveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  // mouseDownCallback() does not necessarily come before maveCallback()
  if(!is_last_valid){
    return;
  }

  gm::Point current_position = feedback->pose.position;
  gm::Quaternion current_orientation = feedback->pose.orientation;
  
  Point3d adjustment = Point3d{current_position.x - last_position_.x, 
                               current_position.y - last_position_.y,
                               current_position.z - last_position_.z};
  prism_->move(adjustment);

  // Rotate polygon
  tf::Quaternion last;
  tf::Quaternion cur;
  tf::quaternionMsgToTF(last_orientation_, last);
  tf::quaternionMsgToTF(current_orientation, cur);

  tf::Quaternion qdiff = cur * last.inverse();
  tfScalar diff = qdiff.getAngle();
  tf::Vector3 axes = qdiff.getAxis();
  double d_alpha = diff * axes.getZ();
  prism_->rotate(d_alpha);

  last_orientation_ = current_orientation;
  last_position_ = current_position;
}

void CenterControl::mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  last_orientation_ = feedback->pose.orientation;
  last_position_ = feedback->pose.position;
  is_last_valid = true;
}

void CenterControl::mouseUpCallback( [[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  is_last_valid = false;
}

void CenterControl::deleteCallback( [[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  safety_zone_->deleteObstacle(obstacle_id_);
}


} // namespace mrs_lib