#include <mrs_lib/SafetyZone/SafetyZone.h>
#include <mrs_lib/SafetyZone/lineOperations.h>

namespace mrs_lib
{
/* SafetyZone() //{ */

SafetyZone::SafetyZone(mrs_lib::Polygon border, std::vector<Polygon> innerObstacles, std::vector<PointObstacle> pointObstacles)
    : innerObstacles(innerObstacles), pointObstacles(pointObstacles) {
  outerBorder = new Polygon(border);
}

//}

/* SafetyZone() //{ */

SafetyZone::~SafetyZone() {
  delete outerBorder;
}

//}

/* SafetyZone() //{ */

SafetyZone::SafetyZone(const Eigen::MatrixXd& outerBorderMatrix, const std::vector<Eigen::MatrixXd>& innerObstaclesMatrixes,
                       const std::vector<Eigen::MatrixXd>& pointObstaclesMatrixes) {
  try {
    outerBorder = new Polygon(outerBorderMatrix);
  }
  catch (Polygon::WrongNumberOfVertices) {
    throw BorderError();
  }
  catch (Polygon::WrongNumberOfColumns) {
    throw BorderError();
  }
  catch (Polygon::ExtraVertices) {
    throw BorderError();
  }

  for (auto& matrix : innerObstaclesMatrixes) {
    try {
      innerObstacles.emplace_back(matrix);
    }
    catch (Polygon::WrongNumberOfVertices) {
      throw PolygonObstacleError();
    }
    catch (Polygon::WrongNumberOfColumns) {
      throw PolygonObstacleError();
    }
    catch (Polygon::ExtraVertices) {
      throw PolygonObstacleError();
    }
  }

  for (auto& matrix : pointObstaclesMatrixes) {
    if (matrix.cols() != 3 || matrix.rows() != 1) {
      throw PointObstacleError();
    }

    pointObstacles.emplace_back(Eigen::RowVector2d{matrix(0, 0), matrix(0, 1)}, matrix(0, 2));
  }
}

//}

/* isPointValid() //{ */

bool SafetyZone::isPointValid(const double px, const double py) {

  if (!outerBorder->isPointInside(px, py)) {
    return false;
  }

  for (auto& elem : innerObstacles) {
    if (elem.isPointInside(px, py)) {
      return false;
    }
  }

  for (auto& elem : pointObstacles) {
    if (elem.isPointInside(px, py)) {
      return false;
    }
  }

  return true;
}

//}

/* isPathValid() //{ */

bool SafetyZone::isPathValid(const double p1x, const double p1y, const double p2x, const double p2y) {

  if (outerBorder->doesSectionIntersect(p1x, p1y, p2x, p2y))
    return false;

  for (auto& el : innerObstacles) {
    if (el.doesSectionIntersect(p1x, p1y, p2x, p2y))
      return false;
  }

  for (auto& el : pointObstacles) {
    if (el.doesSectionIntersect(p1x, p1y, p2x, p2y))
      return false;
  }

  return true;
}

//}

/* getBorder() //{ */

Polygon SafetyZone::getBorder() {
  return *outerBorder;
}

//}

/* getObstacles() //{ */

std::vector<Polygon> SafetyZone::getObstacles() {
  return innerObstacles;
}

//}

/* getPointObstacles() //{ */

std::vector<PointObstacle> SafetyZone::getPointObstacles() {
  return pointObstacles;
}

//}

}  // namespace mrs_lib
