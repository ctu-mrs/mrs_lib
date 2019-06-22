//
// Created by markiian on 20.6.19.
//

#ifndef MRS_LIB_SAFETYZONE_H
#define MRS_LIB_SAFETYZONE_H

#include <mrs_lib/SafetyZone/PointObstacle.h>
#include <mrs_lib/SafetyZone/Polygon.h>

namespace mrs_lib {
    class SafetyZone {
    public:
        SafetyZone(Polygon outerBorder, std::vector<Polygon> innerObstacles, std::vector<PointObstacle> pointObstacles);
        bool isPointValid(const double px,  const double py);
        Polygon getBorder();
        std::vector<Polygon> getObstacles();
        std::vector<PointObstacle> getPointObstacles();

    private:
        Polygon outerBorder;
        std::vector<Polygon> innerObstacles;
        std::vector<PointObstacle> pointObstacles;
    };
}

#endif //MRS_LIB_SAFETYZONE_H
