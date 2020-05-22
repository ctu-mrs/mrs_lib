#include <mrs_lib/safety_zone/point_obstacle.h>
#include <mrs_lib/safety_zone/line_operations.h>
#include <ros/ros.h>

#include <math.h>

namespace mrs_lib
{

/* PointObstacle() //{ */

PointObstacle::PointObstacle(const Eigen::RowVector2d center, const double r, const double height) : center(center), r(r), height(height) {
  if (r <= 0) {
    ROS_WARN("(Point Obstacle) Radius must be positive");
    throw WrongRadius();
  }

  if (height <= 0) {
    ROS_WARN("(Point Obstacle) Height must be positive");
    throw WrongHeight();
  }
}

//}

/* isPointInside3d() //{ */

bool PointObstacle::isPointInside3d(const double px, const double py, const double pz) {

  return (pz <= height) && (pow(px - center(0), 2.0) + pow(py - center(1), 2.0)) < pow(r, 2.0);
}

//}

/* isPointInside2d() //{ */

bool PointObstacle::isPointInside2d(const double px, const double py) {

  return (pow(px - center(0), 2.0) + pow(py - center(1), 2.0)) < pow(r, 2.0);
}

//}

/* doesSectionIntersect3d() //{ */

bool PointObstacle::doesSectionIntersect3d(const double startX, const double startY, const double startZ, const double endX, const double endY,
                                           const double endZ) {

  Eigen::RowVector2d start{startX, startY};
  Eigen::RowVector2d end{endX, endY};

  Eigen::RowVector2d orthogonal_vector{end(1) - start(1), start(0) - end(0)};

  orthogonal_vector.normalize();

  Eigen::RowVector2d circleDiagStart = center - orthogonal_vector * r;
  Eigen::RowVector2d circleDiagEnd   = center + orthogonal_vector * r;

  bool intersects_2d = sectionIntersect(start, end, circleDiagStart, circleDiagEnd).intersect;

  if (intersects_2d) {

    if (startZ < height || endZ < height) {

      return true;

    } else {

      return false;
    }

  } else {

    return false;
  }
}

//}

/* doesSectionIntersect2d() //{ */

bool PointObstacle::doesSectionIntersect2d(const double startX, const double startY, const double endX, const double endY) {

  Eigen::RowVector2d start{startX, startY};
  Eigen::RowVector2d end{endX, endY};

  Eigen::RowVector2d orthogonal_vector{end(1) - start(1), start(0) - end(0)};

  orthogonal_vector.normalize();

  Eigen::RowVector2d circleDiagStart = center - orthogonal_vector * r;
  Eigen::RowVector2d circleDiagEnd   = center + orthogonal_vector * r;

  return sectionIntersect(start, end, circleDiagStart, circleDiagEnd).intersect;
}

//}

/* inflateSelf() //{ */

void PointObstacle::inflateSelf(double amount) {

  r += amount;
  height += amount;
}

//}

/* getPointMessageVector() //{ */

std::vector<geometry_msgs::Point> PointObstacle::getPointMessageVector(const double z) {

  double new_z = z;

  if (z < 0) {
    new_z = height;
  }

  std::vector<geometry_msgs::Point> points(8);

  for (int i = 0; i < 8; ++i) {
    points[i].x = center(0);
    points[i].y = center(1);
    points[i].z = new_z;
  }

  double cos45 = 0.7071067811;

  points[0].y += r;
  points[0].z = new_z;

  points[1].x += r * cos45;
  points[1].y += r * cos45;
  points[1].z = new_z;

  points[2].x += r;
  points[2].z = new_z;

  points[3].x += r * cos45;
  points[3].y -= r * cos45;
  points[3].z = new_z;

  points[4].y -= r;
  points[4].z = new_z;

  points[5].x -= r * cos45;
  points[5].y -= r * cos45;
  points[5].z = new_z;

  points[6].x -= r;
  points[6].z = new_z;

  points[7].x -= r * cos45;
  points[7].y += r * cos45;
  points[7].z = new_z;

  return points;
}

//}

}  // namespace mrs_lib
