#include <mrs_lib/safety_zone/safety_zone.h>
#include <mrs_lib/safety_zone/line_operations.h>

namespace mrs_lib
{
/* SafetyZone() //{ */

SafetyZone::SafetyZone(mrs_lib::Polygon border) {
  outerBorder = new Polygon(border);
}

//}

/* SafetyZone() //{ */

SafetyZone::~SafetyZone() {
  delete outerBorder;
}

//}

/* SafetyZone() //{ */

SafetyZone::SafetyZone(const Eigen::MatrixXd& outerBorderMatrix) {

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

}  // namespace mrs_lib
