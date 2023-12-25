#ifndef MRS_LIB_SAFETYZONE_H
#define MRS_LIB_SAFETYZONE_H

#include <vector>
#include <map>

#include "prism.h"
#include "polygon.h"
#include "yaml_export_visitor.h"

namespace mrs_lib
{
class SafetyZone {
public:
  // TODO: add constructor from parameters or smth like this
  SafetyZone(Prism outer_border);
  SafetyZone(Prism outer_broder, std::vector<Prism>& obstacles);

  // Controls, if 3d point lies within the prism
  bool isPointValid(const Point3d point);

  // Convinient version of isPointIn(Point3d point)
  bool isPointValid(const double px, const double py, const double pz);

  // Controls, if 2d point lies within the polygon of prism (i.e. ignores max_z and min_z of all prisms)
  bool isPointValid(const Point2d point);

  // Convinient version of isPointIn(Point2d point)
  bool isPointValid(const double px, const double py);

  // The function divides the path into smaller segments (on average 20 segments per meter) 
  // and validates each intermediate point to determine if the entire path is valid.
  bool isPathValid(const Point3d start, const Point3d end);

  // 2d version of isPathValid(const Point3d start, const Point3d end), i.e. ignores max_z and min_z of all prisms
  bool isPathValid(const Point2d start, const Point2d end);

  Prism& getBorder() {
    return outer_border_;
  }

  Prism& getObstacle(int index) {
    return obstacles_.at(index);
  }

  std::map<int, Prism>::iterator getObstaclesBegin(){
    return obstacles_.begin();
  }

  std::map<int, Prism>::iterator getObstaclesEnd(){
    return obstacles_.end();
  }

  void addObstacle(int id, Prism obstacle){
    // TODO: this does not add, it changes!!!
    obstacles_.at(id) = obstacle;
  }

  void deleteObstacle(int id){
    obstacles_.erase(id);
  }

  // Helper method for text representation
  void accept(Visitor& visitor); 

private:
  Prism outer_border_;
  std::map<int, Prism> obstacles_;
};
}  // namespace mrs_lib

#endif  // MRS_LIB_SAFETYZONE_H
