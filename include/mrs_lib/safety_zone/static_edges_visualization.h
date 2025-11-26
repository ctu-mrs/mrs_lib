#ifndef STATIC_EDGES_VISUALIZATION_H
#define STATIC_EDGES_VISUALIZATION_H

#include "prism.h"
#include <mrs_lib/safety_zone.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mrs_lib/publisher_handler.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <atomic>

namespace mrs_lib
{
  class StaticEdgesVisualization final : public safety_zone::Subscriber
  {

  public:
    // Represents the prism
    StaticEdgesVisualization(safety_zone::Prism prism, const std::string& uav_name, const std::string& frame_id, const rclcpp::Node::SharedPtr node,
                             const double& markers_update_rate);

    // Represents corresponding obstacle in the safety_zone.
    StaticEdgesVisualization(safety_zone::SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name, const std::string& frame_id,
                             const rclcpp::Node::SharedPtr node, const double& markers_update_rate);

    // Represents border of the safety_zone.
    StaticEdgesVisualization(safety_zone::SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id, const rclcpp::Node::SharedPtr node,
                             const double& markers_update_rate);

    ~StaticEdgesVisualization();

    void update() override;
    void cleanup() override;

  private:
    void init();
    void sendMarker();

    // It is required for generating static marker names,
    // because their id's must be unique on topic
    const int id_;
    static std::atomic<int> id_generator_;

    safety_zone::Prism prism_;
    std::string uav_name_;
    std::string frame_id_;
    rclcpp::Node::SharedPtr node_;

    // Static markers
    mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> publisher_;
    mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> coordinate_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    visualization_msgs::msg::MarkerArray last_markers_;
    visualization_msgs::msg::MarkerArray last_coordinates_;

  }; // class StaticEdgesVisualization
} // namespace mrs_lib


#endif
