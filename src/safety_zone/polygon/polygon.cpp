#include "mrs_lib/SafetyZone/Polygon.h"


namespace mrs_lib {
    Polygon::Polygon(const Eigen::MatrixXd vertices):vertices(vertices) {
        if (vertices.cols() != 2) {
            ROS_WARN("(Polygon) The supplied polygon has to have 2 cols. It has %lu.", vertices.cols());
            throw WrongNumberOfColumns();
        }
        if (vertices.rows() < 3) {
            ROS_WARN("(Polygon) The supplied polygon has to have at least 3 vertices. It has %lu.", vertices.rows());
            throw WrongNumberOfVertices();
        }

    }

    bool Polygon::isPointInside(const double px, const double py) {
        int count = 0;
        // Cast a horizontal ray and see how many times it intersects all the edges
        for (int i = 0; i < vertices.rows(); ++i) {
            double v1x = vertices(i, 0);
            double v1y = vertices(i, 1);
            double v2x = vertices((i + 1) % vertices.rows(), 0);
            double v2y = vertices((i + 1) % vertices.rows(), 1);

            if (v1y > py && v2y > py) continue;
            if (v1y <= py && v2y <= py) continue;
            double intersect_x = (v1y == v2y) ? v1x : (py - v1y) * (v2x - v1x) / (v2y - v1y) + v1x;
            bool does_intersect = intersect_x > px;
            if (does_intersect) ++count;
        }

        return count % 2;
    }
}