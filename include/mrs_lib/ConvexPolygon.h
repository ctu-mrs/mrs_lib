#ifndef CONVEXPOLYGON_H
#define CONVEXPOLYGON_H

/* author: Tomas Baca */

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <mutex>

namespace mrs_lib
{

class ConvexPolygon {

public:
  ConvexPolygon(const Eigen::MatrixXd vert);

private:
  int    n;  // number of vertices
  double total_area;

  Eigen::MatrixXd vert;  // n*2

  std::mutex polygon_mutex;

private:
  double triangleArea(const Eigen::VectorXd a, const Eigen::VectorXd b, const Eigen::VectorXd c);
  double convexPolygonArea(void);

public:
  // checks if the counter-clockwise-defined polygon is convex
  bool isConvex(void);

  bool isPointIn(double px, double py);

public:
  // exceptions
  struct PolygonNotConvexException : public std::exception
  {
    const char* what() const throw() {
      return "ConvexPolygon: Polygon is not convex!";
    }
  };

  struct WrongNumberOfVertices : public std::exception
  {
    const char* what() const throw() {
      return "ConvexPolygon: wrong number of vertices was supplied!";
    }
  };

  struct WrongNumberOfColumns : public std::exception
  {
    const char* what() const throw() {
      return "ConvexPolygon: wrong number of colums, it should be =2!";
    }
  };
};

}  // namespace mrs_lib

#endif
