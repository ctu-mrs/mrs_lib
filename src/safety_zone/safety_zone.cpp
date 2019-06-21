#include <mrs_lib/SafetyZone/SafetyZone.h>


namespace mrs_lib {
    SafetyZone::SafetyZone(mrs_lib::Polygon & outerBorder, std::vector<mrs_lib::Polygon> & innerObstacles,
                           std::vector<mrs_lib::PointObstacle> & pointObstacles):outerBorder(outerBorder), innerObstacles(innerObstacles), pointObstacles(pointObstacles) {}

    bool SafetyZone::isPointValid(const double px, const double py) {
        if (!outerBorder.isPointInside(px, py)) {
            return false;
        }
        for (auto & elem: innerObstacles) {
            if (elem.isPointInside(px, py)) {
                return false;
            }
        }

        for (auto & elem: pointObstacles) {
            if (elem.isPointInside(px, py)) {
                return false;
            }
        }
        return true;
    }
}