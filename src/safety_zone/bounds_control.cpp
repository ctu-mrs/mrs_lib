#include "mrs_lib/safety_zone/bounds_control.h"

#include <tf/tf.h>
#include <cmath>
#include <sstream>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

enum class BoundaryType
{
  UPPER,
  LOWER

};

namespace mrs_lib
{
  std::atomic<int> BoundsControl::id_generator = 0;

  /* BoundsControl() //{ */

  BoundsControl::BoundsControl(Prism* prism, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh)
      : id_(id_generator++), prism_(prism), uav_name_(uav_name), frame_id_(frame_id), nh_(nh)
  {
    init();
  }

  //}

  /* BoundsControl() //{ */

  BoundsControl::BoundsControl(SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh)
      : id_(id_generator++), obstacle_id_(obstacle_id), prism_(safety_zone->getObstacle(obstacle_id)), uav_name_(uav_name), frame_id_(frame_id), nh_(nh), safety_zone_(safety_zone)
  {
    init();
  }

  //}

  /* BoundsControl() //{ */

  BoundsControl::BoundsControl(SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh)
      : id_(id_generator++), prism_(safety_zone->getBorder()), uav_name_(uav_name), frame_id_(frame_id), nh_(nh)
  {
    init();
  }

  //}

  /* ~BoundsControl() //{ */

  BoundsControl::~BoundsControl()
  {
    if (is_active_)
    {
      prism_->unsubscribe(this);
    }
    cleanup();
  }

  //}

  /* init() //{ */

  void BoundsControl::init()
  {
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(nh_.getNamespace() + "/safety_area_bounds_out", std::to_string(id_), false);
    menu_handler_ = std::make_unique<interactive_markers::MenuHandler>();
    if (safety_zone_)
    {
      menu_handler_->insert("Delete the prism", [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->deleteCallback(feedback); });
    } else
    {
      menu_handler_->insert("Deleting is disabled for this prism");
    }

    prism_->subscribe(this);

    addBoundIntMarker(BoundaryType::UPPER);
    addBoundIntMarker(BoundaryType::LOWER);

    is_active_ = true;  // flag inherited from Subscriber class, defined in Prism
  }

  //}

  /* update() //{ */

  void BoundsControl::update()
  {
    if (!is_active_)
    {
      return;
    }
    gm::Pose pose;
    pose.position.x = prism_->getCenter().get<0>();
    pose.position.y = prism_->getCenter().get<1>();
    pose.position.z = prism_->getMaxZ();

    // Upper marker
    vm::InteractiveMarker current_marker;
    server_->get(upper_name_, current_marker);
    current_marker.controls[2].markers[0] = makeText(pose.position.z);
    server_->insert(current_marker);
    server_->setPose(upper_name_, pose);

    // Lower marker
    pose.position.z = prism_->getMinZ();
    server_->get(lower_name_, current_marker);
    current_marker.controls[2].markers[0] = makeText(pose.position.z);
    server_->insert(current_marker);
    server_->setPose(lower_name_, pose);

    server_->applyChanges();
  }

  //}

  /* cleanup() //{ */

  void BoundsControl::cleanup()
  {
    server_->clear();
    server_->applyChanges();
    is_active_ = false;
  }

  //}

  /* makeBox() //{ */

  vm::Marker BoundsControl::makeBox(const vm::InteractiveMarker& msg)
  {
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

  //}

  /* makeText() //{ */

  vm::Marker BoundsControl::makeText(const double& value)
  {
    vm::Marker marker;

    marker.type = vm::Marker::TEXT_VIEW_FACING;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 3;

    std::stringstream ss;
    ss << std::setprecision(2) << std::fixed << frame_id_ << ":" << std::endl << "Z: " << value;

    marker.text = ss.str();

    return marker;
  }

  //}

  /* addBoundIntMarker() //{ */

  void BoundsControl::addBoundIntMarker(const BoundaryType& boundary_type)
  {
    const Point2d center = prism_->getCenter();
    // Interactive marker
    vm::InteractiveMarker int_marker;
    std::string target_frame_id = "world_origin";
    int_marker.header.frame_id = uav_name_ + "/" + target_frame_id;
    /* int_marker.header.stamp.fromNSec(0); */
    int_marker.header.stamp = ros::Time::now();
    int_marker.pose.position.x = center.get<0>();
    int_marker.pose.position.y = center.get<1>();
    int_marker.scale = 3;

    if (boundary_type == BoundaryType::UPPER)
    {
      int_marker.pose.position.z = prism_->getMaxZ();
      upper_name_ = std::to_string(id_) + "_upper";
      int_marker.name = upper_name_;
    } else
    {
      int_marker.pose.position.z = prism_->getMinZ();
      lower_name_ = std::to_string(id_) + "_lower";
      int_marker.name = lower_name_;
    }

    // Control
    vm::InteractiveMarkerControl control;
    tf::Quaternion orientation(0.0, 1.0, 0.0, 1.0);
    orientation.normalize();
    tf::quaternionTFToMsg(orientation, control.orientation);

    // Z Moving element
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // XY Moving box
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;
    int_marker.controls.push_back(control);

    // Text
    control.interaction_mode = vm::InteractiveMarkerControl::NONE;
    control.orientation_mode = vm::InteractiveMarkerControl::VIEW_FACING;
    control.markers.clear();
    control.markers.push_back(makeText(int_marker.pose.position.z));
    int_marker.controls.push_back(control);

    // Send to server
    server_->insert(int_marker);
    menu_handler_->apply(*server_, int_marker.name);
    server_->setCallback(
        int_marker.name, [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->boundMoveCallback(feedback); },
        vm::InteractiveMarkerFeedback::POSE_UPDATE);
    server_->setCallback(
        int_marker.name, [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->mouseDownCallback(feedback); },
        vm::InteractiveMarkerFeedback::MOUSE_DOWN);
    server_->setCallback(
        int_marker.name, [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->mouseUpCallback(feedback); },
        vm::InteractiveMarkerFeedback::MOUSE_UP);
    server_->applyChanges();
    server_->applyChanges();
  }

  //}

  /* boundMoveCallback() //{ */

  void BoundsControl::boundMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    double z = feedback->pose.position.z;
    z = round(z * 5) / 5;
    if (feedback->marker_name == upper_name_)
    {
      prism_->setMaxZ(z);
    } else if (feedback->marker_name == lower_name_)
    {
      prism_->setMinZ(z);
    } else
    {
      ROS_WARN("[BoundsControl]: unknown marker appeared %s", feedback->marker_name.c_str());
    }

    if (!is_last_valid_)
    {
      return;
    }

    const gm::Point current_position = feedback->pose.position;
    const Point3d adjustment = Point3d{current_position.x - last_position_.x, current_position.y - last_position_.y, 0};
    prism_->move(adjustment);
    last_position_ = current_position;
  }

  //}

  /* mouseDownCallback() //{ */

  void BoundsControl::mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    last_position_ = feedback->pose.position;
    is_last_valid_ = true;
  }

  //}

  /* mouseUpCallback() //{ */

  void BoundsControl::mouseUpCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    is_last_valid_ = false;
  }

  //}

  /* deleteCallback() //{ */

  void BoundsControl::deleteCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    safety_zone_->deleteObstacle(obstacle_id_);
  }

  //}

}  // namespace mrs_lib
