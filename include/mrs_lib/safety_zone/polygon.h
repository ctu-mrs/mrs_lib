#ifndef MRS_LIB_POLYGON_H_
#define MRS_LIB_POLYGON_H_

#include <boost/geometry.hpp>

#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace mrs_lib
{
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> Point2d;

typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> Point3d;

typedef boost::geometry::model::polygon<Point2d> Polygon;

} // namespace mrs_lib


#endif  // MRS_LIB_POLYGON_H
