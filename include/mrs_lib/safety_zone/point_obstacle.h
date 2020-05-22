// clang: TomasFormat
#ifndef MRS_LIB_POINTOBSTACLE_H
#define MRS_LIB_POINTOBSTACLE_H

#include <exception>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>

namespace mrs_lib
{
class PointObstacle {
public:
  PointObstacle(const Eigen::RowVector2d center, const double r, const double height);
  bool isPointInside3d(const double px, const double py, const double pz);
  bool isPointInside2d(const double px, const double py);
  bool doesSectionIntersect3d(const double startX, const double startY, const double startZ, const double endX, const double endY, const double endZ);
  bool doesSectionIntersect2d(const double startX, const double startY, const double endX, const double endY);
  void inflateSelf(double amount);
  std::vector<geometry_msgs::Point> getPointMessageVector(const double z);

private:
  Eigen::RowVector2d center;
  double             r;
  double             height;

public:
  // exceptions
  struct WrongRadius : public std::exception
  {
    const char* what() const throw() {
      return "PointObstacle: radius must be > 0!";
    }
  };
  struct WrongHeight : public std::exception
  {
    const char* what() const throw() {
      return "PointObstacle: height must be > 0!";
    }
  };
};
}  // namespace mrs_lib


#endif  // MRS_LIB_POINTOBSTACLE_H
