#include <mrs_lib/safety_zone.h>

namespace bg = boost::geometry;

namespace mrs_lib
{

  namespace safety_zone
  {

    /* SafetyZone() //{ */
    // Constructor without obstacles
    SafetyZone::SafetyZone(std::unique_ptr<Prism>&& outer_border) : outer_border_(std::move(outer_border))
    {
    }

    //}

    /* SafetyZone() //{ */
    // Constructor with obstacles
    SafetyZone::SafetyZone(std::unique_ptr<Prism>&& outer_border, std::vector<std::unique_ptr<Prism>>&& obstacles) : outer_border_(std::move(outer_border))
    {

      for (auto& obstacle : obstacles)
      {
        obstacles_.emplace(next_obstacle_id_++, std::move(obstacle));
      }
    }
    //}

    /* enableSafetyZone() //{ */
    void SafetyZone::enableSafetyZone(const bool enable)
    {
      std::scoped_lock lock(mutex_safety_zone_);
      safety_zone_enabled_ = enable;
    }
    //}

    /* safetyZoneEnabled() //{ */
    bool SafetyZone::safetyZoneEnabled(void)
    {
      std::scoped_lock lock(mutex_safety_zone_);
      return safety_zone_enabled_;
    }
    //}

    /* isPointValid(Point2d) //{ */
    bool SafetyZone::isPointValid(const Point2d& point)
    {
      std::scoped_lock lock(mutex_safety_zone_);

      if (!safety_zone_enabled_)
      {
        return true;
      }

      if (!outer_border_->isPointIn(point))
      {
        return false;
      }

      for (auto& obstacle : obstacles_)
      {
        if (obstacle.second->isPointIn(point))
        {
          return false;
        }
      }

      return true;
    }
    //}

    /* isPointValid(px, py) //{ */
    bool SafetyZone::isPointValid(const double px, const double py)
    {
      std::scoped_lock lock(mutex_safety_zone_);

      if (!safety_zone_enabled_)
      {
        return true;
      }

      if (!outer_border_->isPointIn(px, py))
      {
        return false;
      }

      for (auto& obstacle : obstacles_)
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
    // Overload for Point3d
    bool SafetyZone::isPointValid(const Point3d& point)
    {
      std::scoped_lock lock(mutex_safety_zone_);

      if (!safety_zone_enabled_)
      {
        return true;
      }

      if (!outer_border_->isPointIn(point))
      {
        return false;
      }

      for (auto& obstacle : obstacles_)
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
      std::scoped_lock lock(mutex_safety_zone_);

      if (!safety_zone_enabled_)
      {
        return true;
      }

      if (!outer_border_->isPointIn(px, py, pz))
      {
        return false;
      }

      for (auto& obstacle : obstacles_)
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
    bool SafetyZone::isPathValid(const Point3d& start, const Point3d& end)
    {

      if (!safety_zone_enabled_)
      {
        return true;
      }

      int count = static_cast<int>(ceil((bg::distance(start, end) * _discretization_steps_)));

      Point3d current_point = start;
      Point3d increment = end;
      bg::subtract_point(increment, start); // Calculate a vector from start to end
      bg::divide_value(increment, count);   // Obtain the incremental step vector
      for (int i = 0; i < count; i++)
      {
        if (!isPointValid(current_point))
        {
          return false;
        }
        bg::add_point(current_point,
                      increment); // Advancing current point by the step vector to
                                  // move along the path
      }
      return true;
    }
    //}

    /* isPathValid() //{ */

    bool SafetyZone::isPathValid(const Point2d& start, const Point2d& end)
    {

      if (!safety_zone_enabled_)
      {
        return true;
      }

      int count = static_cast<int>(ceil((bg::distance(start, end) * _discretization_steps_)));

      Point2d current_point = start;
      Point2d increment = end;
      bg::subtract_point(increment, start); // Calculate a vector from start to end
      bg::divide_value(increment, count);   // Obtain the incremental step vector
      for (int i = 0; i < count; i++)
      {
        if (!isPointValid(current_point))
        {
          return false;
        }
        bg::add_point(current_point, increment); // Advancing current point by step
                                                 // vector to move along the path
      }
      return true;
    }

    //}

    /* accept() //{ */

    // void SafetyZone::accept(Visitor &visitor) {
    //   visitor.visit(this);
    //
    //   for (auto &entry : obstacles_) {
    //     entry.second->accept(visitor);
    //   }
    // }

    //}

    /* getBorder() //{ */
    Prism SafetyZone::getBorder() const
    {
      std::scoped_lock lock(mutex_safety_zone_);
      return *outer_border_;
    }
    //}

    /* getObstacles() //{ */
    const std::map<int, std::unique_ptr<Prism>>& SafetyZone::getObstacles() const
    {
      std::scoped_lock lock(mutex_safety_zone_);
      return obstacles_;
    }
    //}

    /* getObstacle() //{ */
    Prism SafetyZone::getObstacle(const int index) const
    {
      std::scoped_lock lock(mutex_safety_zone_);
      auto it = obstacles_.find(index);

      if (it == obstacles_.end())
      {
        throw std::out_of_range("No obstacle with the given index");
      } else
      {
        return *(it->second);
      }
    }
    //}

    /* addObstacle //{ */
    int SafetyZone::addObstacle(std::unique_ptr<Prism> obstacle)
    {
      std::scoped_lock lock(mutex_safety_zone_);
      int current_id = ++next_obstacle_id_;
      obstacles_.emplace(current_id, std::move(obstacle));
      return current_id;
    }
    //}

    /* deleteObstacle() //{ */
    void SafetyZone::deleteObstacle(int id)
    {
      std::scoped_lock lock(mutex_safety_zone_);
      // Check if obstacle with given id exists
      auto it = obstacles_.find(id);
      if (it == obstacles_.end())
      {
        throw std::out_of_range("No obstacle with the given id");
      } else
      {
        obstacles_.erase(it);
      }
    }
    //}

  } // namespace safety_zone

} // namespace mrs_lib
