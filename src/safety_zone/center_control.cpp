#include "mrs_lib/safety_zone/center_control.h"

#include <tf/tf.h>
#include <numeric>
#include <math.h>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;
namespace bg = boost::geometry;

namespace mrs_lib
{
  std::atomic<int> CenterControl::id_generator_ = 0;

  /* CenterControl() //{ */
  
  CenterControl::CenterControl(Prism* prism, const std::string& frame_id, const ros::NodeHandle& nh)
      : id_(id_generator_++), prism_(prism), frame_id_(frame_id), nh_(nh)
  {
    init();
  }
  
  //}

  /* CenterControl() //{ */

  CenterControl::CenterControl(SafetyZone* safety_zone, const int& obstacle_id, const std::string& frame_id, const ros::NodeHandle& nh)
      : id_(id_generator_++), prism_(safety_zone->getObstacle(obstacle_id)), obstacle_id_(obstacle_id), frame_id_(frame_id), nh_(nh), safety_zone_(safety_zone)
  {
    init();
    std::cout << "Center vis of obstacle inited\n";
  }

  //}

  /* CenterControl() //{ */

  CenterControl::CenterControl(SafetyZone* safety_zone, const std::string& frame_id, const ros::NodeHandle& nh)
      : id_(id_generator_++), prism_(safety_zone->getBorder()), frame_id_(frame_id), nh_(nh)
  {
    init();
  }

  //}

  /* ~CenterControl() //{ */

  CenterControl::~CenterControl()
  {
    if (is_active_)
    {
      prism_->unsubscribe(this);
    }
    cleanup();
    if (server_)
    {
      delete server_;
    }
    if (menu_handler_)
    {
      delete menu_handler_;
    }
  }

  //}

  /* init() //{ */

  void CenterControl::init()
  {
    server_ = new interactive_markers::InteractiveMarkerServer(nh_.getNamespace() + "/safety_area_center_out", std::to_string(id_), false);
    menu_handler_ = new interactive_markers::MenuHandler();
    if (safety_zone_)
    {
      menu_handler_->insert("Delete the prism", [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->deleteCallback(feedback); });
    } else
    {
      menu_handler_->insert("Deleting is disabled for this prism");
    }

    prism_->subscribe(this);
    addIntMarker();
    is_active_ = true; //flag inherited from Subscriber class (defined in prism) 
  }

  //}

  /* update() //{ */

  void CenterControl::update()
  {
    if (!is_active_)
    {
      return;
    }

    gm::Pose pose;
    pose.position.x = prism_->getCenter().get<0>();
    pose.position.y = prism_->getCenter().get<1>();
    pose.position.z = (prism_->getMaxZ() + prism_->getMinZ()) / 2;

    server_->setPose(marker_name_, pose);
    server_->applyChanges();
  }

  //}

  /* cleanup() //{ */

  void CenterControl::cleanup()
  {
    std::cout << "center cleanup called " << id_ << std::endl;
    server_->clear();
    server_->applyChanges();
    is_active_ = false;
  }

  //}

  /* makeBox() //{ */

  vm::Marker CenterControl::makeBox(vm::InteractiveMarker& msg)
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

  /* addIntMarker() //{ */

  void CenterControl::addIntMarker()
  {
    const Point2d center = prism_->getCenter();
    // Interactive marker
    vm::InteractiveMarker interactive_marker;
    interactive_marker.header.frame_id = frame_id_;
    /* interactive_marker.header.stamp.fromNSec(0); */
    interactive_marker.header.stamp = ros::Time::now(); 
    interactive_marker.pose.position.x = center.get<0>();
    interactive_marker.pose.position.y = center.get<1>();
    interactive_marker.pose.position.z = (prism_->getMaxZ() + prism_->getMinZ()) / 2;
    interactive_marker.scale = 3;
    interactive_marker.name = std::to_string(id_);

    // Control
    vm::InteractiveMarkerControl control;
    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);

    // Rotating around z-axes
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker.controls.push_back(control);

    // Along z axes
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);

    // Along x-y axes (box)
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.push_back(makeBox(interactive_marker));
    control.always_visible = true;
    interactive_marker.controls.push_back(control);

    // Send to server
    server_->insert(interactive_marker);
    menu_handler_->apply(*server_, interactive_marker.name);
    server_->setCallback(
        interactive_marker.name, [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->moveCallback(feedback); },
        vm::InteractiveMarkerFeedback::POSE_UPDATE);
    server_->setCallback(
        interactive_marker.name, [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->mouseDownCallback(feedback); },
        vm::InteractiveMarkerFeedback::MOUSE_DOWN);
    server_->setCallback(
        interactive_marker.name, [this](const vm::InteractiveMarkerFeedbackConstPtr& feedback) { this->mouseUpCallback(feedback); },
        vm::InteractiveMarkerFeedback::MOUSE_UP);
    server_->applyChanges();
    marker_name_ = interactive_marker.name;
  }

  //}

  /* moveCallback() //{ */

  void CenterControl::moveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    // mouseDownCallback() does not necessarily come before maveCallback()
    if (!is_last_valid_)
    {
      return;
    }

    const gm::Point current_position = feedback->pose.position;
    const gm::Quaternion current_orientation = feedback->pose.orientation;

    Point3d adjustment = Point3d{current_position.x - last_position_.x, current_position.y - last_position_.y, current_position.z - last_position_.z};
    prism_->move(adjustment);

    // Rotate polygon
    tf::Quaternion last;
    tf::Quaternion current;
    tf::quaternionMsgToTF(last_orientation_, last);
    tf::quaternionMsgToTF(current_orientation, current);

    tf::Quaternion quaternion_diff = current * last.inverse();
    const tfScalar diff = quaternion_diff.getAngle();
    const tf::Vector3 axes = quaternion_diff.getAxis();
    double alpha_diff = diff * axes.getZ();
    prism_->rotate(alpha_diff);

    last_orientation_ = current_orientation;
    last_position_ = current_position;
  }

  //}

  /* mouseDownCallback() //{ */

  void CenterControl::mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    last_orientation_ = feedback->pose.orientation;
    last_position_ = feedback->pose.position;
    is_last_valid_ = true;
  }

  //}

  /* mouseUpCallback() //{ */

  void CenterControl::mouseUpCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    is_last_valid_ = false;
  }

  //}

  /* deleteCallback() //{ */

  void CenterControl::deleteCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    safety_zone_->deleteObstacle(obstacle_id_);
  }

  //}

}  // namespace mrs_lib
