#include "mrs_lib/Polygon.h"


namespace mrs_lib {
    Polygon::Polygon(const Eigen::MatrixXd & vertices):vertices(vertices) {
        if (vertices.cols() != 2) {
            ROS_WARN("(Outer Border) The supplied polygon has to have 2 cols. It has %lu.", vertices.cols());
            throw WrongNumberOfColumns();
        }
        if (vertices.rows() < 3) {
            ROS_WARN("(Outer Border) The supplied polygon has to have at least 3 vertices. It has %lu.", vertices.rows());
            throw WrongNumberOfVertices();
        }

    }

    bool Polygon::isPointInside(const double px, const double py) {
        int count = 0;

        // Cast a horizontal ray and see how many times it intersects all the edges
        for (int i = 0; i < vertices.rows(); ++i) {
            if (vertices(i, 1) > py && vertices(i + 1, 1) > py) return false;
            if (vertices(i, 1) <= py && vertices(i + 1, 1) <= py) return false;
            double intersect_x = (py - vertices(i, 1)) * (vertices(i + 1, 0) - vertices(i, 0)) / (vertices(i + 1, 1) - vertices(i, 1)) + vertices(i, 0);
            bool does_intersect = intersect_x > px;
            if (does_intersect) ++count;
        }

        return count % 2;
    }
}