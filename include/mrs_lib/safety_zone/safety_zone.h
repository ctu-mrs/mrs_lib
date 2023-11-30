#ifndef MRS_LIB_SAFETYZONE_H
#define MRS_LIB_SAFETYZONE_H

#include <vector>

#include "prism.h"
#include "polygon.h"

namespace mrs_lib
{
class SafetyZone {
public:
  // TODO: add constructor from parameters or smth like this
  SafetyZone(Prism outer_border);
  SafetyZone(Prism outer_broder, std::vector<Prism> obstacles);

  // Controls, if 3d point lies within the prism
  bool isPointValid(const Point3d point);

  // Convinient version of isPointIn(Point3d point)
  bool isPointValid(const double px, const double py, const double pz);

  // Controls, if 2d point lies within the polygon of prism
  bool isPointValid(const Point2d point);

  // Convinient version of isPointIn(Point2d point)
  bool isPointValid(const double px, const double py);

  // The function divides the path into smaller segments (on average 20 segments per meter) 
  // and validates each intermediate point to determine if the entire path is valid.
  bool isPathValid(const Point3d start, const Point3d end);

  Prism& getBorder() {
    return outer_border_;
  }

  Prism& getObstacle(const unsigned int index) {
    return obstacles_[index];
  }

  std::vector<Prism>::iterator getObstaclesBegin(){
    return obstacles_.begin();
  }

  std::vector<Prism>::iterator getObstaclesEnd(){
    return obstacles_.end();
  }

  void addObstacle(Prism obstacle){
    obstacles_.push_back(obstacle);
  }

  void deleteObstacle(std::vector<Prism>::iterator pos){
    obstacles_.erase(pos);
  }

private:
  Prism outer_border_;
  std::vector<Prism> obstacles_;
};
}  // namespace mrs_lib

#endif  // MRS_LIB_SAFETYZONE_H
