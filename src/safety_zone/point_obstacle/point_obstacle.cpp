// clang: MatousFormat
#include <mrs_lib/SafetyZone/PointObstacle.h>
#include <mrs_lib/SafetyZone/lineOperations.h>
#include <ros/ros.h>

#include <math.h>

namespace mrs_lib
{

  /* PointObstacle() //{ */

  PointObstacle::PointObstacle(const Eigen::RowVector2d center, const double r) : center(center), r(r)
  {
    if (r <= 0)
    {
      ROS_WARN("(Point Obstacle) Radius must be non zero");
      throw WrongRadius();
    }
  }

  //}

  /* isPointInside() //{ */

  bool PointObstacle::isPointInside(const double px, const double py)
  {
    return (pow(px - center(0), 2) + pow(py - center(1), 2)) < pow(r, 2);
  }

  //}

  /* doesSectionIntersect() //{ */

  bool PointObstacle::doesSectionIntersect(const double startX, const double startY, const double endX, const double endY)
  {
    Eigen::RowVector2d start{startX, startY};
    Eigen::RowVector2d end{endX, endY};

    Eigen::RowVector2d orthogonal_vector{end(1) - start(1), start(0) - end(0)};
    Eigen::RowVector2d circleDiagStart = center - orthogonal_vector;
    Eigen::RowVector2d circleDiagEnd = center + orthogonal_vector;

    return sectionIntersect(start, end, circleDiagStart, circleDiagEnd).intersect;
  }

  //}

  /* inflateSelf() //{ */

  void PointObstacle::inflateSelf(double amount)
  {
    r += amount;
  }

  //}

  /* getPointMessageVector() //{ */

  std::vector<geometry_msgs::Point> PointObstacle::getPointMessageVector()
  {
    std::vector<geometry_msgs::Point> points(8);

    for (int i = 0; i < 8; ++i)
    {
      points[i].x = center(0);
      points[i].y = center(1);
    }

    double cos45 = 0.7071067811;
    points[0].y += r;
    points[1].x += r * cos45;
    points[1].y += r * cos45;

    points[2].x += r;
    points[3].x += r * cos45;
    points[3].y -= r * cos45;

    points[4].y -= r;
    points[5].x -= r * cos45;
    points[5].y -= r * cos45;

    points[6].x -= r;
    points[7].x -= r * cos45;
    points[7].y += r * cos45;

    return points;
  }

  //}

}  // namespace mrs_lib
