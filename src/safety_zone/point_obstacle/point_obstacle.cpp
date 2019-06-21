#include <mrs_lib/SafetyZone/PointObstacle.h>
#include <ros/ros.h>

#include <math.h>

namespace mrs_lib {
    PointObstacle::PointObstacle(const Eigen::Vector2d center, const double r) : center(center), r(r) {
        if (r <= 0) {
        ROS_WARN("(Point Obstacle) Radius must be non zero");
            throw WrongRadius();
        }
    }

    bool PointObstacle::isPointInside(const double px,  const double py) {
        return (pow(px - center(0),2) + pow(py - center(1),2)) < pow(r,2);

    }

}