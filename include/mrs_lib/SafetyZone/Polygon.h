//
// Created by markiian on 20.6.19.
//

#ifndef MRS_LIB_POLYGON_H
#define MRS_LIB_POLYGON_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <geometry_msgs/Polygon.h>

namespace mrs_lib
{
    class Polygon {
    public:
        Polygon(const Eigen::MatrixXd vertices);
        bool isPointInside(const double px,  const double py);
        geometry_msgs::Polygon get_ros_message();

    private:
        const Eigen::MatrixXd vertices;

    public:
        // exceptions
        struct WrongNumberOfVertices : public std::exception
        {
            const char* what() const throw() {
                return "Polygon: wrong number of vertices was supplied!";
            }
        };

        struct WrongNumberOfColumns : public std::exception
        {
            const char* what() const throw() {
                return "Polygon: wrong number of colums, it should be =2!";
            }
        };
    };
}

#endif //MRS_LIB_POLYGON_H
