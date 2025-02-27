#ifndef BATCH_VISUALIZER__VISUAL_OBJECT_H
#define BATCH_VISUALIZER__VISUAL_OBJECT_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mrs_lib/geometry/shapes.h>
#include <mrs_msgs/msg/path.hpp>
#include <mrs_msgs/msg/trajectory_reference.hpp>

#define DEFAULT_ELLIPSE_POINTS 64

namespace mrs_lib
{

enum MarkerType
{
  POINT    = 0,
  LINE     = 1,
  TRIANGLE = 2
};

class VisualObject {


public:
  VisualObject(const Eigen::Vector3d& point, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const unsigned long& id, const rclcpp::Node::SharedPtr& node);

  VisualObject(const mrs_lib::geometry::Ray& ray, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const unsigned long& id, const rclcpp::Node::SharedPtr& node);

  VisualObject(const mrs_lib::geometry::Triangle& triangle, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node);

  VisualObject(const mrs_lib::geometry::Rectangle& rectangle, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node);

  VisualObject(const mrs_lib::geometry::Cuboid& cuboid, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node);

  VisualObject(const mrs_lib::geometry::Ellipse& ellipse, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node, const int num_points = DEFAULT_ELLIPSE_POINTS);

  VisualObject(const mrs_lib::geometry::Cylinder& cylinder, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const bool filled, const bool capped, const unsigned long& id, const rclcpp::Node::SharedPtr& node, const int num_sides = DEFAULT_ELLIPSE_POINTS);

  VisualObject(const mrs_lib::geometry::Cone& cone, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const bool filled, const bool capped, const unsigned long& id, const rclcpp::Node::SharedPtr& node, const int num_sides = DEFAULT_ELLIPSE_POINTS);

  VisualObject(const mrs_msgs::msg::Path& p, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled,
               const unsigned long& id, const rclcpp::Node::SharedPtr& node);

  VisualObject(const mrs_msgs::msg::TrajectoryReference& traj, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout,
               const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node);


public:
  unsigned long getID() const;
  int           getType() const;
  bool          isTimedOut() const;

  const std::vector<geometry_msgs::msg::Point> getPoints() const;
  const std::vector<std_msgs::msg::ColorRGBA>  getColors() const;

  bool operator<(const VisualObject& other) const {
    return id_ < other.id_;
  }

  bool operator>(const VisualObject& other) const {
    return id_ > other.id_;
  }

  bool operator==(const VisualObject& other) const {
    return id_ == other.id_;
  }

private:
  const unsigned long               id_;
  MarkerType                        type_;
  std::vector<geometry_msgs::msg::Point> points_;
  std::vector<std_msgs::msg::ColorRGBA>  colors_;
  rclcpp::Time                         timeout_time_;
  rclcpp::Node::SharedPtr node_;

  void addRay(const mrs_lib::geometry::Ray& ray, const double r, const double g, const double b, const double a);

  void addTriangle(const mrs_lib::geometry::Triangle& triangle, const double r, const double g, const double b, const double a, const bool filled);

  void addEllipse(const mrs_lib::geometry::Ellipse& ellipse, const double r, const double g, const double b, const double a, const bool filled,
                  const int num_points);

};  // namespace batch_visualizer

}  // namespace mrs_lib

#endif
