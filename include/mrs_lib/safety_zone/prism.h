#ifndef MRS_PRISM_H
#define MRS_PRISM_H

#include "polygon.h"
#include <boost/geometry/algorithms/centroid.hpp>

namespace mrs_lib
{

class Subscriber{
public:
  virtual void update() = 0;
};

class Prism{
private:
  Polygon polygon_;
  double max_z_;
  double min_z_;

  std::set<Subscriber*> subscribers;

public:
  // If max_z < min_z, automatically swaps them
  Prism(Polygon& polygon, double max_z, double min_z);

  void subscribe(Subscriber* entity);
  void unsubscribe(Subscriber* entity);
  void notifySubscribers();

  double getMaxZ(){
    return max_z_;
  }

  double getMinZ(){
    return min_z_;
  }

  Polygon getPolygon(){
    return polygon_;
  }

  // Returns number of vertices in the polygon of the prism.
  unsigned int getVerticesNum(){
    return polygon_.outer().size() - 1;
  }

  // Returns the centroid of the polygon of the prism.
  Point2d getCenter(){
    Point2d res;
    boost::geometry::centroid(polygon_, res);
    return res;
  }

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
  bool setVerticies(std::vector<Point2d>& vertices, std::vector<unsigned int> indices);

  // Adds new vertex in the middle of the neighboring verge
  void addVertexClockwise(unsigned int index);

  // Adds new vertex in the middle of the neighboring verge
  void addVertexCounterclockwise(unsigned int index);

  // Controls, if 3d point lies within the prism 
  bool isPointIn(Point3d point);

  // Convinient version of isPointIn(Point3d point)
  bool isPointIn(double x, double y, double z);

  // Controls, if 2d point lies within the polygon of prism
  bool isPointIn(Point2d point);
  
  // Convinient version of isPointIn(Point2d point)
  bool isPointIn(double x, double y);
};

} // namespace mrs_lib

#endif