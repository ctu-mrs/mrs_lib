#include <mrs_lib/safety_zone.h>

namespace mrs_lib
{

namespace safety_zone
{

/* SafetyZone() //{ */

SafetyZone::SafetyZone(std::shared_ptr<Polygon> border) {
  outerBorder = border;
}

//}

/* SafetyZone() //{ */

SafetyZone::SafetyZone(const Eigen::MatrixXd& outerBorderMatrix) {

  try {
    outerBorder = std::make_shared<Polygon>(outerBorderMatrix);
  }
  catch (const WrongNumberOfVertices&) {
    throw BorderError();
  }
  catch (const WrongNumberOfColumns&) {
    throw BorderError();
  }
  catch (const ExtraVertices&) {
    throw BorderError();
  }
}

//}

/* isPointValid() //{ */

bool SafetyZone::isPointValid(const double px, const double py) {

  if (!outerBorder->isPointInside(px, py)) {
    return false;
  }

  return true;
}

//}

/* isPathValid() //{ */

bool SafetyZone::isPathValid(const double p1x, const double p1y, const double p2x, const double p2y) {

  if (outerBorder->doesSectionIntersect(p1x, p1y, p2x, p2y)) {
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

}  // namespace safety_zone

}  // namespace mrs_lib
