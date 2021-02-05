#include <mrs_lib/safety_zone/safety_zone.h>
#include <mrs_lib/safety_zone/line_operations.h>

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
  catch (const Polygon::WrongNumberOfVertices&) {
    throw BorderError();
  }
  catch (const Polygon::WrongNumberOfColumns&) {
    throw BorderError();
  }
  catch (const Polygon::ExtraVertices&) {
    throw BorderError();
  }

  for (auto& matrix : innerObstaclesMatrixes) {
    try {
      innerObstacles.emplace_back(matrix);
    }
    catch (const Polygon::WrongNumberOfVertices&) {
      throw PolygonObstacleError();
    }
    catch (const Polygon::WrongNumberOfColumns&) {
      throw PolygonObstacleError();
    }
    catch (const Polygon::ExtraVertices&) {
      throw PolygonObstacleError();
    }
  }

  for (auto& matrix : pointObstaclesMatrixes) {
    if (matrix.cols() != 4 || matrix.rows() != 1) {
      throw PointObstacleError();
    }

    pointObstacles.emplace_back(Eigen::RowVector2d{matrix(0, 0), matrix(0, 1)}, matrix(0, 2), matrix(0, 3));
  }
}

//}

/* isPointValid3d() //{ */

bool SafetyZone::isPointValid3d(const double px, const double py, const double pz) {

  if (!outerBorder->isPointInside(px, py)) {
    return false;
  }

  for (auto& elem : innerObstacles) {
    if (elem.isPointInside(px, py)) {
      return false;
    }
  }

  for (auto& elem : pointObstacles) {
    if (elem.isPointInside3d(px, py, pz)) {
      return false;
    }
  }

  return true;
}

//}

/* isPointValid2d() //{ */

bool SafetyZone::isPointValid2d(const double px, const double py) {

  if (!outerBorder->isPointInside(px, py)) {
    return false;
  }

  for (auto& elem : innerObstacles) {
    if (elem.isPointInside(px, py)) {
      return false;
    }
  }

  for (auto& elem : pointObstacles) {
    if (elem.isPointInside2d(px, py)) {
      return false;
    }
  }

  return true;
}

//}

/* isPathValid3d() //{ */

bool SafetyZone::isPathValid3d(const double p1x, const double p1y, const double p1z, const double p2x, const double p2y, const double p2z) {

  if (outerBorder->doesSectionIntersect(p1x, p1y, p2x, p2y)) {
    return false;
  }

  for (auto& el : innerObstacles) {
    if (el.doesSectionIntersect(p1x, p1y, p2x, p2y)) {
      return false;
    }
  }

  for (auto& el : pointObstacles) {
    if (el.doesSectionIntersect3d(p1x, p1y, p1z, p2x, p2y, p2z)) {
      return false;
    }
  }

  return true;
}

//}

/* isPathValid2d() //{ */

bool SafetyZone::isPathValid2d(const double p1x, const double p1y, const double p2x, const double p2y) {

  if (outerBorder->doesSectionIntersect(p1x, p1y, p2x, p2y)) {
    return false;
  }

  for (auto& el : innerObstacles) {
    if (el.doesSectionIntersect(p1x, p1y, p2x, p2y)) {
      return false;
    }
  }

  for (auto& el : pointObstacles) {
    if (el.doesSectionIntersect2d(p1x, p1y, p2x, p2y)) {
      return false;
    }
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
