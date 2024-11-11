#ifndef MRS_LIB_SAFETYZONE_H
#define MRS_LIB_SAFETYZONE_H

#include <vector>
#include <map>

#include "prism.h"
#include "polygon.h"
#include "yaml_export_visitor.h"
#include <unordered_map>
#include <memory>

namespace mrs_lib
{
  class SafetyZone
  {
  public:
    SafetyZone(Prism outer_border);

    // Cleaning the obstacles' memory is SafetyZone's responsibility
    SafetyZone(Prism outer_border, std::vector<std::unique_ptr<Prism>> obstacles);

    ~SafetyZone() = default;

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

    Prism* getBorder();

    const std::map<int, std::unique_ptr<Prism>>& getObstacles() const;

    Prism* getObstacle(const int index);

    std::map<int, std::unique_ptr<Prism>>::iterator getObstaclesBegin();

    std::map<int, std::unique_ptr<Prism>>::iterator getObstaclesEnd();

    int addObstacle(std::unique_ptr<Prism> obstacle);

    void deleteObstacle(const int id);

    // Helper method for text representation
    void accept(Visitor& visitor);

  private:
    Prism outer_border_;
    std::map<int, std::unique_ptr<Prism>> obstacles_;
    std::mutex mutex_safety_zone_;
    int next_obstacle_id_ = 0;

    //Used to discretize between start and end for the path validation
    int _discretization_steps_ = 20; 
  };  // class SafetyZone

}  // namespace mrs_lib

#endif  // MRS_LIB_SAFETYZONE_H
