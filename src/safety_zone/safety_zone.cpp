#include "mrs_lib/safety_zone/safety_zone.h"
#include "mrs_lib/safety_zone/polygon.h"
#include "math.h"

namespace bg = boost::geometry;

namespace mrs_lib
{

SafetyZone::SafetyZone(Prism outer_boarder) : outer_border_(outer_boarder){
}

SafetyZone::SafetyZone(Prism outer_boarder, std::vector<Prism> obstacles) : outer_border_(outer_boarder){
  for(int i=0; i<obstacles.size(); i++) {
    obstacles_.insert({i, obstacles[i]});
    // obstacles_[i] = obstacles[i];
  }
}

bool SafetyZone::isPointValid(const Point2d point) {
  if(!outer_border_.isPointIn(point)){
    return false;
  }

  for (auto obstacle : obstacles_) {
    if(obstacle.second.isPointIn(point)) {
      return false;
    }
  }

  return true;
}

bool SafetyZone::isPointValid(const double px, const double py) {
  if(!outer_border_.isPointIn(px, py)){
    return false;
  }

  for (auto obstacle : obstacles_) {
    if(obstacle.second.isPointIn(px, py)) {
      return false;
    }
  }

  return true;
}

bool SafetyZone::isPointValid(const Point3d point) {
  if(!outer_border_.isPointIn(point)){
    return false;
  }

  for (auto obstacle : obstacles_) {
    if(obstacle.second.isPointIn(point)) {
      return false;
    }
  }

  return true;
}

bool SafetyZone::isPointValid(const double px, const double py, const double pz) {
  if(!outer_border_.isPointIn(px, py, pz)){
    return false;
  }

  for (auto obstacle : obstacles_) {
    if(obstacle.second.isPointIn(px, py, pz)) {
      return false;
    }
  }

  return true;
}

bool SafetyZone::isPathValid(const Point3d start, const Point3d end) {
  int count = (int)ceil((bg::distance(start, end) * 20));

  Point3d cur_point = start;
  Point3d ds = end;
  bg::subtract_point(ds, start);
  bg::divide_value(ds, count);
  for(int i=0; i<count; i++){
    if(!isPointValid(cur_point)){
      return false;
    }
    bg::add_point(cur_point, ds);
  }
  return true;
}

void SafetyZone::accept(Visitor& visitor) {
  visitor.visit(this);

  for(auto& entry : obstacles_) {
    entry.second.accept(visitor);
  }
}
}  // namespace mrs_lib
