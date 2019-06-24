//
// Created by markiian on 20.6.19.
//

#ifndef MRS_LIB_SAFETYZONE_H
#define MRS_LIB_SAFETYZONE_H

#include <mrs_lib/SafetyZone/PointObstacle.h>
#include <mrs_lib/SafetyZone/Polygon.h>
#include <visualization_msgs/Marker.h>

namespace mrs_lib {
    class SafetyZone {
    public:
        SafetyZone(Polygon outerBorder, std::vector<Polygon> innerObstacles, std::vector<PointObstacle> pointObstacles);
        bool isPointValid(const double px,  const double py);
        bool isPathValid(const double p1x, const double p1y, const double p2x, const double p2y);
        Polygon getBorder();
        std::vector<Polygon> getObstacles();
        std::vector<PointObstacle> getPointObstacles();
        visualization_msgs::Marker getMarkerMessage();

    private:
        Polygon outerBorder;
        std::vector<Polygon> innerObstacles;
        std::vector<PointObstacle> pointObstacles;
    };
}

#endif //MRS_LIB_SAFETYZONE_H
