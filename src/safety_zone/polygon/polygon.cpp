#include "mrs_lib/SafetyZone/Polygon.h"
#include "mrs_lib/SafetyZone/lineOperations.h"


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

        Eigen::RowVector2d edge1;
        Eigen::RowVector2d edge2 = vertices.row(1) - vertices.row(0);
        for (int i = 0; i < vertices.rows(); ++i) {
            edge1 = edge2;
            edge2 = vertices.row((i + 2) % vertices.rows()) - vertices.row((i + 1) % vertices.rows());

            if (edge1(0) == 0 && edge1(1) == 0) {
                ROS_WARN("(Polygon) Polygon edge %d is of length zero.", i);
                throw ExtraVertices();
            }
            if (edge1(0) * edge2(1) == edge1(1) * edge2(0)) {
                ROS_WARN("(Polygon) Adjacent Polygon edges at vertice %d are collinear.", (int) ((i + 1) % vertices.rows()));
                throw ExtraVertices();
            }
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

    bool Polygon::doesSectionIntersect(const double startX, const double startY, const double endX, const double endY) {
        Eigen::RowVector2d start{startX, startY};
        Eigen::RowVector2d end{endX, endY};

        for (int i = 0; i < vertices.rows(); ++i) {
            Eigen::RowVector2d edgeStart = vertices.row(i);
            Eigen::RowVector2d edgeEnd = vertices.row((i + 1) % vertices.rows());

            if (sectionIntersect(start, end, edgeStart, edgeEnd).intersect) return true;
        }

        return false;
    }


    std::vector<geometry_msgs::Point> Polygon::getPointMessageVector() {
        std::vector<geometry_msgs::Point> points(vertices.rows()); 
        for (int i = 0; i < vertices.rows(); ++i) {
            points[i].x = vertices(i, 0);
            points[i].y = vertices(i, 1);
        }

        return points;
    }
}
