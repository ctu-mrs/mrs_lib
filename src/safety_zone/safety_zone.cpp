#include "mrs_lib/safety_zone/safety_zone.h"
#include "mrs_lib/safety_zone/polygon.h"
#include "math.h"
#include <memory>
#include <stdexcept>

namespace bg = boost::geometry;

namespace mrs_lib
{

  /* SafetyZone() //{ */
  SafetyZone::SafetyZone(Prism outer_border) : outer_border_(outer_border)
  {
  }
  //}

  /* SafetyZone() //{ */
  SafetyZone::SafetyZone(Prism outer_border, std::vector<Prism*> obstacles) : outer_border_(outer_border)
  {

    for (size_t i = 0; i < obstacles.size(); i++)
    {
      obstacles_.insert({next_obstacle_id_, obstacles[i]});
      next_obstacle_id_++;
    }
  }
  //}

  /* ~SafetyZone() //{ */
  SafetyZone::~SafetyZone()
  {
    for (auto obstacle : obstacles_)
    {
      delete obstacle.second;
    }
  }
  //}

  /* isPointValid(Point2d) //{ */

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

  //}

  /* isPointVavlid(px, py) //{ */

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

  //}

  /* isPointValid() //{ */

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

  //}

  /* isPointValid(px, py, pz) //{ */

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

  //}

  /* isPathValid() //{ */

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

  //}

  /* isPathValid() //{ */

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

  //}

  /* accept() //{ */

  void SafetyZone::accept(Visitor& visitor)
  {
    visitor.visit(this);

    for (auto& entry : obstacles_)
    {
      entry.second->accept(visitor);
    }
  }

  //}

  /* getBorder() //{ */

  Prism* SafetyZone::getBorder()
  {
    return &outer_border_;
  }

  //}

  /* getObstacle() //{ */

  Prism* SafetyZone::getObstacle(int index)
  {
    return obstacles_.at(index);
  }

  //}

  /* getObstacleBegin() //{ */

  std::map<int, Prism*>::iterator SafetyZone::getObstaclesBegin()
  {
    return obstacles_.begin();
  }

  //}

  /* getObstacleEnd() //{ */

  std::map<int, Prism*>::iterator SafetyZone::getObstaclesEnd()
  {
    return obstacles_.end();
  }

  //}

  /* addObstacle //{ */

  int SafetyZone::addObstacle(Prism* obstacle)
  {
    obstacles_.insert({++next_obstacle_id_, obstacle});
    return next_obstacle_id_;
  }

  //}

  /* deleteObstacle() //{ */

  void SafetyZone::deleteObstacle(int id)
  {
    delete obstacles_.at(id);
    obstacles_.erase(id);
  }

  //}

}  // namespace mrs_lib
