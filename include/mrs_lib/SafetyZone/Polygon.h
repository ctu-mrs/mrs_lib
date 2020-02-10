#ifndef MRS_LIB_POLYGON_H
#define MRS_LIB_POLYGON_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <geometry_msgs/Point.h>
#include <mrs_lib/SafetyZone/lineOperations.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace mrs_lib
{
class Polygon {
private:
  using point = boost::geometry::model::d2::point_xy<double>;
  using linestring = boost::geometry::model::linestring<point>;
  using ring = boost::geometry::model::ring<point>;
  using polygon = boost::geometry::model::polygon<point>;
  using mpolygon = boost::geometry::model::multi_polygon<polygon>;

public:
  Polygon(const Eigen::MatrixXd vertices);

  bool isPointInside(const double px, const double py);
  bool doesSectionIntersect(const double startX, const double startY, const double endX, const double endY);
  bool isClockwise();
  void inflateSelf(double amount);

  std::vector<geometry_msgs::Point> getPointMessageVector(const double z);

private:
  ring m_ring;

public:
  // exceptions
  struct WrongNumberOfVertices : public std::exception
  {
    const char* what() const throw() {
      return "Polygon: wrong number of vertices was supplied!";
    }
  };

  struct WrongNumberOfColumns : public std::exception
  {
    const char* what() const throw() {
      return "Polygon: wrong number of colums, it should be =2!";
    }
  };

  struct ExtraVertices : public std::exception
  {
    const char* what() const throw() {
      return "Polygon: useless vertices detected, polygon methods may break!";
    }
  };

  struct InvalidInflation : public std::exception
  {
    const char* what() const throw() {
      return "Polygon: inflation broke the polygon!";
    }
  };
};
}  // namespace mrs_lib

#endif  // MRS_LIB_POLYGON_H
