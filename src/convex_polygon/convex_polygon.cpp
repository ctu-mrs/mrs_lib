/* author: Tomas Baca */

#include <ros/ros.h>
#include <mrs_lib/ConvexPolygon.h>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

namespace mrs_lib
{

/* constructor //{ */

ConvexPolygon::ConvexPolygon(const MatrixXd in) {

  if (in.cols() != 2) {
    ROS_WARN("The supplied polygon has to have 2 cols. It has %lu.", in.cols());
    throw WrongNumberOfColumns();
    return;
  }

  if (in.rows() < 3) {
    ROS_WARN("The supplied polygon has to have at least 3 vertices. It has %lu.", in.rows());
    throw WrongNumberOfVertices();
    return;
  }

  n = in.rows() + 1;

  vert                             = MatrixXd::Zero(n, 2);
  this->vert.block(0, 0, n - 1, 2) = in;

  // repeat the last point as the first on
  this->vert.row(n - 1) = in.row(0);

  // calculate the total area of the polygon
  total_area = convexPolygonArea();
}

//}

/* triangleArea() //{ */

double ConvexPolygon::triangleArea(const Eigen::VectorXd a, const Eigen::VectorXd b, const Eigen::VectorXd c) {

  return fabs((a(0) * b(1) + b(0) * c(1) + c(0) * a(1) - a(1) * b(0) - b(1) * c(0) - c(1) * a(0)) / 2.0);
}

//}

/* isPointIn() //{ */

bool ConvexPolygon::isPointIn(const double px, const double py) {

  double area = 0;

  Eigen::VectorXd point = VectorXd::Zero(2);
  point << px, py;

  for (int i = 1; i < n; i++) {
    area += triangleArea(vert.row(i), vert.row(i - 1), point);
  }

  if (area > total_area * 1.001)
    return false;
  else
    return true;
}

//}

/* convexPolygonArea() //{ */

double ConvexPolygon::convexPolygonArea(void) {

  if (!isConvex()) {

    ROS_WARN("The polygon is not convex!");
    throw PolygonNotConvexException();
    return 0;
  }

  // compute the center of gravity
  VectorXd cot = VectorXd::Zero(2);

  for (int i = 0; i < n - 1; i++) {
    cot(0) += vert(i, 0);
    cot(1) += vert(i, 1);
  }

  cot(0) /= vert.rows();
  cot(1) /= vert.rows();

  // compute the area
  double area = 0;

  for (int i = 1; i < n; i++) {
    area += triangleArea(vert.row(i), vert.row(i - 1), cot);
  }

  return area;
}

//}

/* isConvex() //{ */

bool ConvexPolygon::isConvex(void) {

  for (int i = 2; i < n; i++) {

    double angle_1 = fmod(atan2(vert(i - 1, 1) - vert(i - 2, 1), vert(i - 1, 0) - vert(i - 2, 0)), (2.0 * M_PI));
    double angle_2 = fmod(atan2(vert(i, 1) - vert(i - 1, 1), vert(i, 0) - vert(i - 1, 0)), (2.0 * M_PI));

    if (fmod(angle_2 - angle_1, 2.0 * M_PI) > M_PI) {
      return false;
    }
  }

  return true;
}

//}
}  // namespace mrs_lib
