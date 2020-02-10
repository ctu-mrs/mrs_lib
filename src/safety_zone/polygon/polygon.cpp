#include "mrs_lib/SafetyZone/Polygon.h"
#include "mrs_lib/SafetyZone/lineOperations.h"

namespace mrs_lib
{

/* Polygon() //{ */

Polygon::Polygon(const Eigen::MatrixXd vertices)
{
  if (vertices.cols() != 2) {
    ROS_WARN("(Polygon) The supplied polygon has to have 2 cols. It has %lu.", vertices.cols());
    throw WrongNumberOfColumns();
  }
  if (vertices.rows() < 3) {
    ROS_WARN("(Polygon) The supplied polygon has to have at least 3 vertices. It has %lu.", vertices.rows());
    throw WrongNumberOfVertices();
  }

  Eigen::RowVector2d edge1;
  Eigen::RowVector2d edge2 = vertices.row(1) - vertices.row(0);
  for (int i = 0; i < vertices.rows(); ++i) {
    edge1 = edge2;
    edge2 = vertices.row((i + 2) % vertices.rows()) - vertices.row((i + 1) % vertices.rows());

    if (edge1(0) == 0 && edge1(1) == 0) {
      ROS_WARN("(Polygon) Polygon edge %d is of length zero.", i);
      throw ExtraVertices();
    }
    if (edge1(0) * edge2(1) == edge1(1) * edge2(0)) {
      ROS_WARN("(Polygon) Adjacent Polygon edges at vertice %d are collinear.", (int)((i + 1) % vertices.rows()));
      throw ExtraVertices();
    }
  }

  std::vector<point> pts;
  pts.reserve(vertices.rows());
  for (int i = 0; i < vertices.rows(); ++i)
  {
    const Eigen::RowVector2d row = vertices.row(i);
    pts.push_back({row.x(), row.y()});
  }
  pts.push_back({vertices.row(0).x(), vertices.row(0).y()});
  m_ring = ring(std::begin(pts), std::end(pts));
}

//}

/* doesSectionIntersect() //{ */

bool Polygon::doesSectionIntersect(const double startX, const double startY, const double endX, const double endY)
{
  std::vector<point> pts = {{startX, startY}, {endX, endY}};
  linestring line(std::begin(pts), std::end(pts));
  return boost::geometry::intersects(m_ring, line);
}

//}

/* isPointInside() //{ */

bool Polygon::isPointInside(const double px, const double py)
{
  return boost::geometry::within(point(px, py), m_ring);
}

//}

/* inflateSelf() //{ */

void Polygon::inflateSelf(double amount)
{
  // Declare strategies
  const int points_per_circle = 36;
  boost::geometry::strategy::buffer::distance_symmetric distance_strategy(amount);
  boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
  boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
  boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
  boost::geometry::strategy::buffer::side_straight side_strategy;

  mpolygon result;
  // Create the buffer of a multi polygon
  boost::geometry::buffer(m_ring, result,
              distance_strategy, side_strategy,
              join_strategy, end_strategy, circle_strategy);
  if (result.size() != 1)
    throw InvalidInflation();
  polygon poly = result.at(0);
  m_ring = poly.outer();
}

//}

/* getPointMessageVector() //{ */

std::vector<geometry_msgs::Point> Polygon::getPointMessageVector(const double z)
{
  std::vector<geometry_msgs::Point> points;
  points.reserve(m_ring.size());
  for (const auto& pt : m_ring)
  {
    geometry_msgs::Point gpt;
    gpt.x = pt.x();
    gpt.y = pt.y();
    gpt.z = z;
    points.push_back(gpt);
  }
  return points;
}

//}

}  // namespace mrs_lib
