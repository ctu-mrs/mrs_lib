#ifndef MRS_PRISM_H
#define MRS_PRISM_H

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace mrs_lib
{

  namespace safety_zone
  {

    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> Point2d;
    typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point3d;
    typedef boost::geometry::model::polygon<Point2d> Polygon2D;

    class Subscriber
    {
    protected:
      bool is_active_ = true;

    public:
      // Called every time a change has been made to the prism
      virtual void update() = 0;

      // Called once upon deleting the prism
      virtual void cleanup() = 0;
    };

    class Prism
    {
    private:
      Polygon2D polygon_;
      double min_z_;
      double max_z_;
      std::string horizontal_frame_; // frame in which the polygon is defined
      std::string vertical_frame_;   // frame in which the z limits are defined

      std::set<Subscriber*> subscribers_;

      void notifySubscribers();
      void cleanSubscribers();

    public:
      // Empty constructor
      Prism() : polygon_(), min_z_(0.0), max_z_(0.0), horizontal_frame_("world_origin"), vertical_frame_("world_origin")
      {
      }
      // If max_z < min_z, automatically swaps them
      // Throws std::invalid argument if polygon is invalid
      Prism(const std::vector<Point2d>& points, const double max_z, const double min_z);

      Prism(const std::vector<Point2d>& points, const double max_z, const double min_z, const std::string& horizontal_frame, const std::string& vertical_frame);

      // Copy assignment operator
      Prism& operator=(const Prism& other)
      {
        if (this != &other)
        {
          polygon_ = other.polygon_;
          min_z_ = other.min_z_;
          max_z_ = other.max_z_;
          horizontal_frame_ = other.horizontal_frame_;
          vertical_frame_ = other.vertical_frame_;
          // Keep existing subscribers, don't copy them
        }
        return *this;
      }

      // Copy constructor
      Prism(const Prism& other)
          : polygon_(other.polygon_), min_z_(other.min_z_), max_z_(other.max_z_), horizontal_frame_(other.horizontal_frame_),
            vertical_frame_(other.vertical_frame_), subscribers_() // Don't copy observers
      {
      }

      // Move constructor
      Prism(Prism&& other) noexcept
          : polygon_(std::move(other.polygon_)), min_z_(other.min_z_), max_z_(other.max_z_), horizontal_frame_(other.horizontal_frame_),
            vertical_frame_(other.vertical_frame_), subscribers_(std::move(other.subscribers_))
      {
      }

      // Move assignment operator
      Prism& operator=(Prism&& other) noexcept
      {
        if (this != &other)
        {
          polygon_ = std::move(other.polygon_);
          min_z_ = other.min_z_;
          max_z_ = other.max_z_;
          horizontal_frame_ = other.horizontal_frame_;
          vertical_frame_ = other.vertical_frame_;
          subscribers_ = std::move(other.subscribers_);
        }
        return *this;
      }

      ~Prism();

      void subscribe(Subscriber* entity);
      void unsubscribe(Subscriber* entity);

      std::vector<Point2d> getPoints();
      double getMaxZ() const;
      double getMinZ() const;
      std::string getHorizontalFrame() const;
      std::string getVerticalFrame() const;

      Polygon2D getPolygon() const;

      // Returns number of vertices in the polygon of the prism.
      unsigned int getNumVertices() const;

      // Returns the centroid of the polygon of the prism.
      Point2d getCenter() const;

      void setMaxZ(const double value);

      void setMinZ(const double value);

      // Tries to change the coordinates of given vertex.
      // returns true if succeeded
      // returns false otherwise
      bool setVertex(const Point2d& vertex, const unsigned int index);

      // Tries to change the coordinates of given vertices.
      // Only notifies subsribers once in case of success
      // returns true if succeeded
      // returns false otherwise
      bool setVertices(const std::vector<Point2d>& vertices, const std::vector<unsigned int>& indices);

      // Adds new vertex in the middle of the neighboring verge
      void addVertexClockwise(const unsigned int index);

      // Adds new vertex in the middle of the neighboring verge
      void addVertexCounterclockwise(const unsigned int index);

      void move(const Point3d& adjustment);

      // Takes angle in radians, rotates clockwise (counter-clockwise if alpha < 0)
      void rotate(const double alpha);

      // Deletes the vertex only if vertex_count > 3
      void deleteVertex(const unsigned int index);

      // Overload with Point3d
      bool isPointIn(const Point3d& point) const;

      // Overload with 3D point (double,double,double)
      bool isPointIn(const double x, const double y, const double z) const;

      // Overload with Point2d
      bool isPointIn(const Point2d& point) const;

      // Overload with 2D point (double,double)
      bool isPointIn(const double x, const double y) const;

      // Helper method for text representation
      // void accept(Visitor &visitor);
    };

  } // namespace safety_zone
} // namespace mrs_lib

#endif
