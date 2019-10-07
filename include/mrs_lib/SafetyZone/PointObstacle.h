// clang: MatousFormat
//
// Created by markiian on 20.6.19.
//

#ifndef MRS_LIB_POINTOBSTACLE_H
#define MRS_LIB_POINTOBSTACLE_H

#include <exception>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>

namespace mrs_lib
{
  class PointObstacle
  {
  public:
    PointObstacle(const Eigen::RowVector2d center, const double r);
    bool isPointInside(const double px, const double py);
    bool doesSectionIntersect(const double startX, const double startY, const double endX, const double endY);
    void inflateSelf(double amount);
    std::vector<geometry_msgs::Point> getPointMessageVector();

  private:
    Eigen::RowVector2d center;
    double r;

  public:
    // exceptions
    struct WrongRadius : public std::exception
    {
      const char* what() const throw()
      {
        return "PointObstacle: radius must be > 0!";
      }
    };
  };
}  // namespace mrs_lib


#endif  // MRS_LIB_POINTOBSTACLE_H
