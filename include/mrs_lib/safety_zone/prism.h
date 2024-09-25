#ifndef MRS_PRISM_H
#define MRS_PRISM_H

#include "polygon.h"
#include "yaml_export_visitor.h"
#include <boost/geometry/algorithms/centroid.hpp>

namespace mrs_lib
{

class Subscriber{
protected:
  bool is_active_ = true;
public:
  // Called every time a change has been made to the prism
  virtual void update() = 0;

  // Called once upon deleting the prism
  virtual void cleanup() = 0;
};

class Prism{
private:
  Polygon polygon_;
  double min_z_;
  double max_z_;

  std::set<Subscriber*> subscribers;

  void notifySubscribers();
  void cleanSubscribers();
public:
  // If max_z < min_z, automatically swaps them
  // Throws std::invalid argument if polygon is invalid
  Prism(const std::vector<Point2d>& points, double max_z, double min_z);

  ~Prism();

  void subscribe(Subscriber* entity);
  void unsubscribe(Subscriber* entity);

  double getMaxZ();

  double getMinZ();

  Polygon getPolygon();

  // Returns number of vertices in the polygon of the prism.
  unsigned int getVerticesNum();

  // Returns the centroid of the polygon of the prism.
  Point2d getCenter();

  void setMaxZ(double value);

  void setMinZ(double value);

  // Tries to change the coordinates of given vertex. 
  // returns true if succeeded
  // returns false otherwise
  bool setVertex(Point2d vertex, unsigned int index);

  // Tries to change the coordinates of given vertecies. 
  // Only notifies subsribers once in case of success
  // returns true if succeeded
  // returns false otherwise
  bool setVertices(std::vector<Point2d>& vertices, std::vector<unsigned int> indices);

  // Adds new vertex in the middle of the neighboring verge
  void addVertexClockwise(unsigned int index);

  // Adds new vertex in the middle of the neighboring verge
  void addVertexCounterclockwise(unsigned int index);

  void move(Point3d adjustment);

  // Takes angle in radians, rotates clockwise (counter-clockwise if alpha < 0)
  void rotate(double alpha);

  // Deletes the vertex only if vertex_count > 3
  void deleteVertex(unsigned int index);

  // Controls, if 3d point lies within the prism 
  bool isPointIn(Point3d point);

  // Convinient version of isPointIn(Point3d point)
  bool isPointIn(double x, double y, double z);

  // Controls, if 2d point lies within the polygon of prism
  bool isPointIn(Point2d point);
  
  // Convinient version of isPointIn(Point2d point)
  bool isPointIn(double x, double y);

  // Helper method for text representation
  void accept(Visitor& visitor);
};

} // namespace mrs_lib

#endif
