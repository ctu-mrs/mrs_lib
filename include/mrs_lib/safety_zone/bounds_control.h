#ifndef BOUNDS_CONTROL_H
#define BOUNDS_CONTROL_H

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

  //  Creates 2 interactive markers in the center of the prism's polygon:
  // one for maximum height of the prism and the other for minimum height.
  // Both markers can be used for moving entire prism and deleting it (view constructors' docstring)
  // Markers also allow changing correspondings heights
  class BoundsControl final : public Subscriber
  {
  public:
    // Created interactive markers do not have deleting option
    BoundsControl(Prism* prism, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh);
    // Represents corresponding obstacle in the safety_zone.
    // Created interactive markers provide deleting option
    BoundsControl(SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh);
    // Represents border of the safety_zone.
    // Created interactive markers do not have deleting option
    BoundsControl(SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh);

    ~BoundsControl();

    void update() override;
    void cleanup() override;

  private:
    enum class BoundaryType
    {
      UPPER,
      LOWER
    };

    // Tools for convenience
    void init();
    void addBoundIntMarker(const BoundaryType& boundary_type);
    visualization_msgs::Marker makeBox(const visualization_msgs::InteractiveMarker& msg);
    visualization_msgs::Marker makeText(const double& value);

    // Markers' callbacks
    void boundMoveCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void mouseDownCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void mouseUpCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void deleteCallback([[maybe_unused]] const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    // It is required for generating server id's,
    // because their id's must be unique on topic
    const int id_;
    static std::atomic<int> id_generator;
    const int obstacle_id_ = 0;

    // Attributes received in constructor
    Prism* prism_;
    std::string uav_name_;
    std::string frame_id_;
    ros::NodeHandle nh_;
    SafetyZone* safety_zone_ = nullptr;

    // Communication with RVIZ
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_ = nullptr;
    std::unique_ptr<interactive_markers::MenuHandler> menu_handler_ = nullptr;

    // Distinguishing features of 2 markers
    std::string upper_name_;
    std::string lower_name_;

    // Required for moving the prism
    geometry_msgs::Point last_position_;
    bool is_last_valid_ = false;

  };  // class BoundsControl
}  // namespace mrs_lib

#endif
