/**  \file
     \brief Object abstraction for the Batch Visualizer
     \author Petr Štibinger - stibipet@fel.cvut.cz
 */

#ifndef BATCH_VISUALIZER__VISUAL_OBJECT_H
#define BATCH_VISUALIZER__VISUAL_OBJECT_H

#include <Eigen/Core>
#include <ros/time.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <mrs_lib/geometry/shapes.h>
#include <mrs_msgs/TrajectoryReference.h>

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
  VisualObject(const Eigen::Vector3d& point, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const unsigned long& id);

  VisualObject(const mrs_lib::geometry::Ray& ray, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const unsigned long& id);

  VisualObject(const mrs_lib::geometry::Triangle& triangle, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const bool filled, const unsigned long& id);

  VisualObject(const mrs_lib::geometry::Rectangle& rectangle, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const bool filled, const unsigned long& id);

  VisualObject(const mrs_lib::geometry::Cuboid& cuboid, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const bool filled, const unsigned long& id);

  VisualObject(const mrs_lib::geometry::Ellipse& ellipse, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const bool filled, const unsigned long& id, const int num_points = DEFAULT_ELLIPSE_POINTS);

  VisualObject(const mrs_lib::geometry::Cylinder& cylinder, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const bool filled, const bool capped, const unsigned long& id, const int num_sides = DEFAULT_ELLIPSE_POINTS);

  VisualObject(const mrs_lib::geometry::Cone& cone, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const bool filled, const bool capped, const unsigned long& id, const int num_sides = DEFAULT_ELLIPSE_POINTS);

  VisualObject(const mrs_msgs::TrajectoryReference& traj, const double r, const double g, const double b, const double a, const ros::Duration& timeout,
               const bool filled, const unsigned long& id);


public:
  unsigned long getID() const;
  int           getType() const;
  bool          isTimedOut() const;

  const std::vector<geometry_msgs::Point> getPoints() const;
  const std::vector<std_msgs::ColorRGBA>  getColors() const;

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
  std::vector<geometry_msgs::Point> points_;
  std::vector<std_msgs::ColorRGBA>  colors_;
  ros::Time                         timeout_time_;

  void addRay(const mrs_lib::geometry::Ray& ray, const double r, const double g, const double b, const double a);

  void addTriangle(const mrs_lib::geometry::Triangle& triangle, const double r, const double g, const double b, const double a, const bool filled);

  void addEllipse(const mrs_lib::geometry::Ellipse& ellipse, const double r, const double g, const double b, const double a, const bool filled,
                  const int num_points);

};  // namespace batch_visualizer

}  // namespace mrs_lib

#endif
