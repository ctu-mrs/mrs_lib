// clang: MatousFormat
#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace mrs_lib
{

double             angle_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2);
Eigen::AngleAxisd  angleaxis_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance = 1e-9);
Eigen::Quaterniond quaternion_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance = 1e-9);
Eigen::Matrix3d    rotation_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance = 1e-9);

class Ray {
public:
  Ray();
  ~Ray();
  Ray(Eigen::Vector3d origin, Eigen::Vector3d direction, double energy = 0.0);

  double          energy;
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;

  static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo) {
    Eigen::Vector3d origin    = pointFrom;
    Eigen::Vector3d direction = (pointTo - pointFrom);
    /* direction.normalize(); */
    return Ray(origin, direction);
  }
  static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction) {
    return Ray(origin, direction);
  }
};

class Plane {
public:
  Plane();
  ~Plane();
  Plane(Eigen::Vector3d point, Eigen::Vector3d normal);

  Eigen::Vector3d point;
  Eigen::Vector3d normal;

  std::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};


class Rectangle {
public:
  Rectangle();
  ~Rectangle();
  Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);

  Eigen::Vector3d normal_vector;

  Eigen::Matrix3d basis;
  Eigen::Matrix3d projector;

  Plane                        plane;
  std::vector<Eigen::Vector3d> points;

  std::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};

class Cone {

public:
  Cone(Eigen::Vector3d position, Eigen::Vector3d direction, double angle);
  ~Cone();
  Eigen::Vector3d ProjectPoint(Eigen::Vector3d point);

  /* NOT FINISHED //{ */
  std::optional<Eigen::Vector3d> ProjectPointOnPlane(Plane plane, Eigen::Vector3d point);
  //}

private:
  Eigen::Vector3d position, direction;
  double          angle;

  Eigen::MatrixXd cone_axis_projector;
  Ray             cone_ray;
};

double haversin(double angle);
double invHaversin(double angle);

double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);
double solidAngle(double a, double b, double c);
double rectSolidAngle(Rectangle r, Eigen::Vector3d center);

double triangleArea(double a, double b, double c);
double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

}  // namespace mrs_lib

#endif
