#include "mrs_lib/safety_zone/prism.h"

namespace bg = boost::geometry;

namespace mrs_lib
{

  namespace safety_zone
  {

    /* Prism() //{ */
    Prism::Prism(const std::vector<Point2d>& points, const double max_z, const double min_z)
        : polygon_(), min_z_(std::min(min_z, max_z)), max_z_(std::max(min_z, max_z))
    {
      if (points.size() < 3)
        throw std::invalid_argument("A valid polygon must have at least three points.");

      // Append points into polygon
      for (const auto& point : points)
        bg::append(polygon_, point);

      // If first and last point not identical, add first point to close the polygon
      if (!bg::equals(points.front(), points.back()))
        bg::append(polygon_, points.front());

      std::string msg;
      bool is_valid;
      is_valid = bg::is_valid(polygon_, msg);

      // Flip the order of vertices if it has wrong orientation
      if (msg == "Geometry has wrong orientation")
      {
        bg::reverse(polygon_);
        is_valid = bg::is_valid(polygon_, msg);
      }

      if (!is_valid)
      {
        throw std::invalid_argument("The polygon is invalid: " + msg);
      }
    }
    //}

    Prism::Prism(const std::vector<Point2d>& points, const double max_z, const double min_z, const std::string& horizontal_frame,
                 const std::string& vertical_frame)
        : polygon_(), min_z_(std::min(min_z, max_z)), max_z_(std::max(min_z, max_z)), horizontal_frame_(horizontal_frame), vertical_frame_(vertical_frame)
    {
      if (points.size() < 3)
        throw std::invalid_argument("A valid polygon must have at least three points.");
      // Append points into polygon
      for (const auto& point : points)
        bg::append(polygon_, point);

      // If first and last point not identical, add first point to close the polygon
      if (!bg::equals(points.front(), points.back()))
        bg::append(polygon_, points.front());

      std::string msg;
      bool is_valid;
      is_valid = bg::is_valid(polygon_, msg);

      // Flip the order of vertices if it has wrong orientation
      if (msg == "Geometry has wrong orientation")
      {
        bg::reverse(polygon_);
        is_valid = bg::is_valid(polygon_, msg);
      }

      if (!is_valid)
      {
        throw std::invalid_argument("The polygon is invalid: " + msg);
      }
    }

    /* Prism() //{ */
    Prism::~Prism()
    {
    }
    //}

    /* subscribe() //{ */
    void Prism::subscribe(Subscriber* entity)
    {
      subscribers_.emplace(entity);
    }
    //}

    /* unsubscribe() //{ */
    void Prism::unsubscribe(Subscriber* entity)
    {
      subscribers_.erase(entity);
    }
    //}

    /* notifySubscribers() //{ */
    void Prism::notifySubscribers()
    {
      for (auto s : subscribers_)
      {
        s->update();
      }
    }
    //}

    /* cleanSubscribers() //{ */
    void Prism::cleanSubscribers()
    {
      for (auto s : subscribers_)
      {
        s->cleanup();
      }
    }
    //}

    /* setMaxZ() //{ */
    void Prism::setMaxZ(const double value)
    {
      max_z_ = value < min_z_ ? min_z_ : value;
      notifySubscribers();
    }
    //}

    /* setMinZ() //{ */
    void Prism::setMinZ(const double value)
    {
      min_z_ = value > max_z_ ? max_z_ : value;
      notifySubscribers();
    }
    //}

    /* setVertex() //{ */
    bool Prism::setVertex(const Point2d& vertex, const unsigned int index)
    {
      auto& outer_ring = polygon_.outer();
      if (index >= outer_ring.size() - 1)
      { // -1 because the last is the same as the first
        return false;
      }

      Point2d tmp = outer_ring[index];
      outer_ring[index] = vertex;
      if (index == 0)
      {
        outer_ring.back() = vertex;
      }

      if (!bg::is_valid(polygon_))
      {
        outer_ring[index] = tmp;
        if (index == 0)
        {
          outer_ring.back() = tmp;
        }
        return false;
      }

      notifySubscribers();
      return true;
    }
    //}

    /* setVertices() //{ */
    bool Prism::setVertices(const std::vector<Point2d>& vertices, const std::vector<unsigned int>& indices)
    {
      if (vertices.size() != indices.size())
      {
        throw std::invalid_argument("Number of vertices and indices must be equal");
      }

      Polygon2D backup = polygon_;
      auto& outer_ring = polygon_.outer();
      bool success = true;
      for (size_t i = 0; i < vertices.size(); i++)
      {
        Point2d vertex = vertices[i];
        unsigned int index = indices[i];
        if (index >= outer_ring.size() - 1)
        { // -1 because the last is the same as the first
          success = false;
          break;
        }

        outer_ring[index] = vertex;
        if (index == 0)
        {
          outer_ring.back() = vertex;
        }
      }

      if (!bg::is_valid(polygon_))
      {
        success = false;
      }

      if (success)
      {
        notifySubscribers();
      } else
      {
        polygon_ = backup;
      }
      return success;
    }
    //}

    /* addVertexCounterclockwise() //{ */
    void Prism::addVertexCounterclockwise(unsigned int index)
    {
      auto& outer_ring = polygon_.outer();
      if (index >= outer_ring.size() - 1)
      { // -1 because the last is the same as the first
        throw std::invalid_argument("Index is out of bounds");
      }

      unsigned int prev_index = index == 0 ? outer_ring.size() - 2 : index - 1;
      double x1 = bg::get<0>(outer_ring[prev_index]);
      double y1 = bg::get<1>(outer_ring[prev_index]);
      double x2 = bg::get<0>(outer_ring[index]);
      double y2 = bg::get<1>(outer_ring[index]);

      Point2d new_point;
      bg::set<0>(new_point, (x1 + x2) / 2);
      bg::set<1>(new_point, (y1 + y2) / 2);

      outer_ring.insert(outer_ring.begin() + prev_index + 1, new_point);
      notifySubscribers();
    }
    //}

    /* addVertexClockwise() //{ */
    void Prism::addVertexClockwise(const unsigned int index)
    {
      auto& outer_ring = polygon_.outer();
      if (index >= outer_ring.size() - 1)
      { // -1 because the last is the same as the first
        throw std::invalid_argument("Index is out of bounds");
      }

      double x1 = bg::get<0>(outer_ring[index]);
      double y1 = bg::get<1>(outer_ring[index]);
      double x2 = bg::get<0>(outer_ring[index + 1]);
      double y2 = bg::get<1>(outer_ring[index + 1]);

      Point2d new_point;
      bg::set<0>(new_point, (x1 + x2) / 2);
      bg::set<1>(new_point, (y1 + y2) / 2);

      outer_ring.insert(outer_ring.begin() + index + 1, new_point);
      notifySubscribers();
    }
    //}

    /* deleteVertex() //{ */
    void Prism::deleteVertex(const unsigned int index)
    {
      auto& outer_ring = polygon_.outer();
      if (index >= outer_ring.size() - 1)
      { // -1 because the last is the same as the first
        throw std::invalid_argument("Index is out of bounds");
      }

      if (outer_ring.size() <= 4)
      {
        return;
      }

      outer_ring.erase(outer_ring.begin() + index);
      if (index == 0)
      {
        outer_ring.back() = outer_ring[0];
      }
      notifySubscribers();
    }
    //}

    /* move() //{ */

    void Prism::move(const Point3d& adjustment)
    {
      bool do_notify = false;

      double dz = adjustment.get<2>();
      if (dz != 0.0)
      {
        do_notify = true;
        max_z_ += dz;
        min_z_ += dz;
      }

      double dx = adjustment.get<0>();
      double dy = adjustment.get<1>();
      Point2d adjustment2d = Point2d{dx, dy};
      if (dx != 0.0 || dy != 0.0)
      {
        do_notify = true;
        auto& outer_ring = polygon_.outer();
        for (size_t i = 0; i < outer_ring.size(); i++)
        {
          bg::add_point(outer_ring[i], adjustment2d);
        }
      }

      if (do_notify)
      {
        notifySubscribers();
      }
    }

    //}

    /* rotate() //{ */
    void Prism::rotate(const double alpha)
    {
      if (alpha == 0.0)
      {
        return;
      }

      Point2d current_center = getCenter();
      auto& outer_ring = polygon_.outer();
      for (size_t i = 0; i < outer_ring.size(); i++)
      {
        double x1 = outer_ring[i].get<0>();
        double y1 = outer_ring[i].get<1>();
        double x2 = (x1 - current_center.get<0>()) * cos(alpha) - (y1 - current_center.get<1>()) * sin(alpha) + current_center.get<0>();
        double y2 = (x1 - current_center.get<0>()) * sin(alpha) + (y1 - current_center.get<1>()) * cos(alpha) + current_center.get<1>();
        outer_ring[i].set<0>(x2);
        outer_ring[i].set<1>(y2);
      }

      notifySubscribers();
    }
    //}

    /* isPointIn(Point3d point) //{ */
    bool Prism::isPointIn(const Point3d& point) const
    {
      Point2d point2d;
      bg::set<0>(point2d, bg::get<0>(point));
      bg::set<1>(point2d, bg::get<1>(point));
      double z = bg::get<2>(point);

      bool result = bg::within(point2d, polygon_) && min_z_ <= z && z <= max_z_;

      return result;
    }
    //}

    /* isPointIn(double x, double y, double z) //{ */
    bool Prism::isPointIn(const double x, const double y, const double z) const
    {
      // Fill in 3d point and call point3d function
      Point3d point;
      bg::set<0>(point, x);
      bg::set<1>(point, y);
      bg::set<2>(point, z);

      return isPointIn(point);
    }
    //}

    /* isPointIn(Point2d point) //{ */
    bool Prism::isPointIn(const Point2d& point) const
    {
      return bg::within(point, polygon_);
    }
    //}

    /* isPointIn(double x, double y) //{ */
    bool Prism::isPointIn(const double x, const double y) const
    {
      Point2d point;
      bg::set<0>(point, x);
      bg::set<1>(point, y);

      return bg::within(point, polygon_);
    }
    //}

    /* accept() //{ */
    // void Prism::accept(Visitor &visitor) {
    //   visitor.visit(this);
    // }
    //}

    /* getCenter() //{ */
    Point2d Prism::getCenter() const
    {
      Point2d res;
      boost::geometry::centroid(polygon_, res);
      return res;
    }
    //}

    /* getPoints() //{ */
    std::vector<Point2d> Prism::getPoints()
    {
      std::vector<Point2d> points;
      const auto& outer_ring = polygon_.outer();

      points.reserve(outer_ring.size());

      for (const auto& point : outer_ring)
      {
        points.emplace_back(point);
      }

      // Remove the last point because it is identical to the first one
      points.pop_back();

      return points;
    }
    //}

    /* getMaxZ() //{ */
    double Prism::getMaxZ() const
    {
      return max_z_;
    }
    //}

    /* getMinZ //{ */
    double Prism::getMinZ() const
    {
      return min_z_;
    }
    //}

    /* getHorizontalFrame() //{ */
    std::string Prism::getHorizontalFrame() const
    {
      return horizontal_frame_;
    }
    //}

    ///* getVerticalFrame() //{ */
    std::string Prism::getVerticalFrame() const
    {
      return vertical_frame_;
    }
    //}

    /* getPolygon2D() //{ */
    Polygon2D Prism::getPolygon() const
    {
      return polygon_;
    }
    //}

    /* getVerticesNum() //{ */
    unsigned int Prism::getNumVertices() const
    {
      return polygon_.outer().size() - 1;
    }
    //}
  } // namespace safety_zone
} // namespace mrs_lib
