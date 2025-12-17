#include <mrs_lib/safety_zone/polygon.h>
#include <mrs_lib/safety_zone/line_operations.h>

namespace mrs_lib
{

  namespace safety_zone
  {

    /* Polygon() //{ */

    Polygon::Polygon(const Eigen::MatrixXd vertices) : vertices(vertices)
    {

      if (vertices.cols() != 2)
      {
        printf("(Polygon) The supplied polygon has to have 2 cols. It has %lu.", vertices.cols());
        throw WrongNumberOfColumns();
      }
      if (vertices.rows() < 3)
      {
        printf("(Polygon) The supplied polygon has to have at least 3 vertices. It has %lu.", vertices.rows());
        throw WrongNumberOfVertices();
      }

      Eigen::RowVector2d edge1;
      Eigen::RowVector2d edge2 = vertices.row(1) - vertices.row(0);
      for (int i = 0; i < vertices.rows(); ++i)
      {
        edge1 = edge2;
        edge2 = vertices.row((i + 2) % vertices.rows()) - vertices.row((i + 1) % vertices.rows());

        if (edge1(0) == 0 && edge1(1) == 0)
        {
          printf("(Polygon) Polygon edge %d is of length zero.", i);
          throw ExtraVertices();
        }
        if (edge1(0) * edge2(1) == edge1(1) * edge2(0))
        {
          printf("(Polygon) Adjacent Polygon edges at vertice %d are collinear.", (int)((i + 1) % vertices.rows()));
          throw ExtraVertices();
        }
      }
    }

    //}

    /* isPointInside() //{ */

    bool Polygon::isPointInside(const double px, const double py)
    {

      int count = 0;

      // Cast a horizontal ray and see how many times it intersects all the edges
      for (int i = 0; i < vertices.rows(); ++i)
      {
        double v1x = vertices(i, 0);
        double v1y = vertices(i, 1);
        double v2x = vertices((i + 1) % vertices.rows(), 0);
        double v2y = vertices((i + 1) % vertices.rows(), 1);

        if (v1y > py && v2y > py)
          continue;
        if (v1y <= py && v2y <= py)
          continue;
        double intersect_x = (v1y == v2y) ? v1x : (py - v1y) * (v2x - v1x) / (v2y - v1y) + v1x;
        bool does_intersect = intersect_x > px;
        if (does_intersect)
          ++count;
      }

      return count % 2;
    }

    //}

    /* doesSectionIntersect() //{ */

    bool Polygon::doesSectionIntersect(const double startX, const double startY, const double endX, const double endY)
    {

      Eigen::RowVector2d start{startX, startY};
      Eigen::RowVector2d end{endX, endY};

      for (int i = 0; i < vertices.rows(); ++i)
      {

        Eigen::RowVector2d edgeStart = vertices.row(i);
        Eigen::RowVector2d edgeEnd = vertices.row((i + 1) % vertices.rows());

        if (sectionIntersect(start, end, edgeStart, edgeEnd).intersect)
        {
          return true;
        }
      }

      return false;
    }

    //}

    /* isClockwise() //{ */

    bool Polygon::isClockwise()
    {

      int edgeIndex = 0;
      if (vertices(0, 1) == vertices(1, 1))
        edgeIndex = 1;

      // get the midPoint of first edge
      Eigen::RowVector2d center = (vertices.row(edgeIndex) + vertices.row(edgeIndex + 1)) / 2;

      // count the intersections for a horizontal ray starting at center
      int intersection_count = 0;
      for (int i = 0; i < vertices.rows(); ++i)
      {
        if (i == edgeIndex)
          continue;

        Eigen::MatrixXd edge(2, 2);
        edge.row(0) = vertices.row(i);
        edge.row(1) = vertices.row((i + 1) % vertices.rows());

        if ((edge(0, 1) <= center(1) && edge(1, 1) <= center(1)) || (edge(0, 1) > center(1) && edge(1, 1) > center(1)))
          continue;
        double intersect_x = edge(0, 0) + (edge(1, 0) - edge(0, 0)) / (edge(1, 1) - edge(0, 1)) * (center(1) - edge(0, 1));
        if (intersect_x >= center(0))
          ++intersection_count;
      }

      // reverse depending on the direction of edge
      bool clockwise = intersection_count % 2;
      if (vertices(edgeIndex + 1, 1) < vertices(edgeIndex, 0))
        clockwise = !clockwise;
      return clockwise;
    }

    //}

    /* movePoint() //{ */

    static Eigen::RowVector2d movePoint(Eigen::RowVector2d vector, double scale, Eigen::RowVector2d point)
    {

      double length = sqrt(vector(0) * vector(0) + vector(1) * vector(1));
      Eigen::RowVector2d unitary_vector = vector / length;
      return point + scale * unitary_vector;
    }

    //}

    /* lineIntersectGivenVector() //{ */

    static Intersection lineIntersectGivenVector(Eigen::RowVector2d start1, Eigen::RowVector2d vector1, Eigen::RowVector2d start2, Eigen::RowVector2d vector2)
    {

      double cross = vector1(0) * vector2(1) - vector1(1) * vector2(0);
      double difference_cross = (start2(0) - start1(0)) * vector2(1) - (start2(1) - start1(1)) * vector1(0);

      if (cross == 0)
      {
        if (difference_cross == 0)
        {
          return Intersection{true, true, start1};
        }
        return Intersection{false, true};
      }

      double scale1 = difference_cross / cross;
      Eigen::RowVector2d point = start1 + scale1 * vector1;
      return Intersection{true, false, point};
    }

    //}

    /* inflateSelf() //{ */

    void Polygon::inflateSelf(double amount)
    {

      // Also supports negative numbers
      Eigen::MatrixXd result{vertices.rows(), vertices.cols()};

      bool clockwise = isClockwise();
      amount = (clockwise ? 1 : -1) * amount;

      for (int i = 0; i < vertices.rows(); ++i)
      {
        Eigen::RowVector2d pPrev = vertices.row((i + vertices.rows() - 1) % vertices.rows());
        Eigen::RowVector2d pCurrent = vertices.row(i);
        Eigen::RowVector2d pNext = vertices.row((i + 1) % vertices.rows());

        Eigen::RowVector2d vector1 = pCurrent - pPrev;
        Eigen::RowVector2d vector2 = pNext - pCurrent;
        // make them orthogonally counter-clockwise
        Eigen::RowVector2d vectorOrthogonal1{-vector1(1), vector1(0)};
        Eigen::RowVector2d vectorOrthogonal2{-vector2(1), vector2(0)};
        // move the currentPoint in 2 directions
        pPrev = movePoint(vectorOrthogonal1, amount, pCurrent);
        pNext = movePoint(vectorOrthogonal2, amount, pCurrent);

        // add the intersection as a new point
        result.row(i) = lineIntersectGivenVector(pPrev, vector1, pNext, vector2).point;
      }

      vertices = result.replicate(result.rows(), result.cols());
    }

    //}

    /* getPointMessageVector() //{ */

    std::vector<geometry_msgs::msg::Point> Polygon::getPointMessageVector(const double z)
    {
      std::vector<geometry_msgs::msg::Point> points(vertices.rows());

      for (int i = 0; i < vertices.rows(); ++i)
      {
        points[i].x = vertices(i, 0);
        points[i].y = vertices(i, 1);
        points[i].z = z;
      }

      return points;
    }

    //}

  } // namespace safety_zone

} // namespace mrs_lib
