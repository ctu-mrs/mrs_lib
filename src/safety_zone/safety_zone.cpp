#include "mrs_lib/safety_zone/safety_zone.h"
#include "mrs_lib/safety_zone/polygon.h"
#include "math.h"
#include <memory>
#include <stdexcept>

namespace bg = boost::geometry;

namespace mrs_lib
{

  SafetyZone::SafetyZone(Prism outer_border) : outer_border_(std::move(outer_border))
  {
  }

  SafetyZone::SafetyZone(Prism outer_border, std::vector<Prism*> obstacles) : outer_border_(std::move(outer_border))
  {

    for (size_t i = 0; i < obstacles.size(); i++)
    {
      obstacles_.insert({next_obstacle_id_, obstacles[i]});
      next_obstacle_id_++;
    }


  }

  SafetyZone::~SafetyZone()
  {
    obstacles_.clear();
  }

  bool SafetyZone::isPointValid(const Point2d point)
  {
    if (!outer_border_.isPointIn(point))
    {
      return false;
    }

    for (auto obstacle : obstacles_)
    {
      if (obstacle.second->isPointIn(point))
      {
        return false;
      }
    }

    return true;
  }

  bool SafetyZone::isPointValid(const double px, const double py)
  {
    if (!outer_border_.isPointIn(px, py))
    {
      return false;
    }

    for (auto obstacle : obstacles_)
    {
      if (obstacle.second->isPointIn(px, py))
      {
        return false;
      }
    }

    return true;
  }

  bool SafetyZone::isPointValid(const Point3d point)
  {
    if (!outer_border_.isPointIn(point))
    {
      return false;
    }

    for (auto obstacle : obstacles_)
    {
      if (obstacle.second->isPointIn(point))
      {
        return false;
      }
    }

    return true;
  }

  bool SafetyZone::isPointValid(const double px, const double py, const double pz)
  {
    if (!outer_border_.isPointIn(px, py, pz))
    {
      return false;
    }

    for (auto obstacle : obstacles_)
    {
      if (obstacle.second->isPointIn(px, py, pz))
      {
        return false;
      }
    }

    return true;
  }

  bool SafetyZone::isPathValid(const Point3d start, const Point3d end)
  {
    int count = (int)ceil((bg::distance(start, end) * 20));

    Point3d cur_point = start;
    Point3d ds = end;
    bg::subtract_point(ds, start);
    bg::divide_value(ds, count);
    for (int i = 0; i < count; i++)
    {
      if (!isPointValid(cur_point))
      {
        return false;
      }
      bg::add_point(cur_point, ds);
    }
    return true;
  }

  bool SafetyZone::isPathValid(const Point2d start, const Point2d end)
  {
    int count = (int)ceil((bg::distance(start, end) * 20));

    Point2d cur_point = start;
    Point2d ds = end;
    bg::subtract_point(ds, start);
    bg::divide_value(ds, count);
    for (int i = 0; i < count; i++)
    {
      if (!isPointValid(cur_point))
      {
        return false;
      }
      bg::add_point(cur_point, ds);
    }
    return true;
  }

  void SafetyZone::accept(Visitor& visitor)
  {
    visitor.visit(this);

    for (auto& entry : obstacles_)
    {
      entry.second->accept(visitor);
    }
  }

  Prism* SafetyZone::getBorder()
  {
    return &outer_border_;
  }

  Prism* SafetyZone::getObstacle(int index)
  {
    return obstacles_.at(index);
  }

  std::map<int, Prism*>::iterator SafetyZone::getObstaclesBegin()
  {
    return obstacles_.begin();
  }

  std::map<int, Prism*>::iterator SafetyZone::getObstaclesEnd()
  {
    return obstacles_.end();
  }

  int SafetyZone::addObstacle(Prism* obstacle)
  {
    obstacles_.insert({++next_obstacle_id_, obstacle});
    return next_obstacle_id_;
  }

  void SafetyZone::deleteObstacle(int id)
  {
    delete obstacles_.at(id);
    obstacles_.erase(id);
  }

}  // namespace mrs_lib
