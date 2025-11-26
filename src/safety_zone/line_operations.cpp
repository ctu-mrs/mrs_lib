#include <mrs_lib/safety_zone/line_operations.h>

namespace mrs_lib
{

  namespace safety_zone
  {

    Intersection::Intersection(bool intersect, bool parallel, Eigen::RowVector2d point) : point(std::move(point)), parallel(parallel), intersect(intersect)
    {
    }

    /* getScale() //{ */

    static double getScale(Eigen::RowVector2d start, Eigen::RowVector2d vector, Eigen::RowVector2d point)
    {
      // Returns the scalar that produces: start + scale * vector = point
      // Assuming that such scalar exists
      return vector(0) != 0 ? (point(0) - start(0)) / vector(0) : (point(1) - start(1)) / vector(1);
    }

    //}

    /* sectionIntersect() //{ */

    Intersection sectionIntersect(Eigen::RowVector2d start1, Eigen::RowVector2d end1, Eigen::RowVector2d start2, Eigen::RowVector2d end2)
    {

      Eigen::RowVector2d vector1 = end1 - start1;
      Eigen::RowVector2d vector2 = end2 - start2;

      // x - cross product (in 2d is just a scalar of z)
      // start1 + scale1 * vector1 = start2 + scale2 * vector2  // x vector2
      // scale1 * vector1 x vector2 = (start2 - start1) x vector2

      // cross product
      double cross = vector1(0) * vector2(1) - vector1(1) * vector2(0);
      double difference_cross = (start2(0) - start1(0)) * vector2(1) - (start2(1) - start1(1)) * vector2(0);

      if (cross == 0)
      {
        // are parallel
        if (difference_cross != 0)
          return Intersection{false, true};

        // are collinear
        double start2Scale = getScale(start1, vector1, start2);
        if (0 <= start2Scale && start2Scale <= 1)
          return Intersection{true, true, start2};
        double end2Scale = getScale(start1, vector1, end2);
        if (0 <= end2Scale && end2Scale <= 1)
          return Intersection{true, true, end2};

        return Intersection{false, true};
      }

      double scale1 = difference_cross / cross;
      Eigen::RowVector2d point = start1 + scale1 * vector1;
      double scale2 = getScale(start2, vector2, point);

      if (0 <= scale1 && scale1 <= 1 && 0 <= scale2 && scale2 <= 1)
      {
        return Intersection{true, false, point};
      }
      return Intersection{false, false};
    }

    //}

  } // namespace safety_zone

} // namespace mrs_lib
