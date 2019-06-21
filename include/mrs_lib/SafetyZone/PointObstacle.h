//
// Created by markiian on 20.6.19.
//

#ifndef MRS_LIB_POINTOBSTACLE_H
#define MRS_LIB_POINTOBSTACLE_H

#include <exception>
#include "eigen3/Eigen/Eigen"

namespace mrs_lib
{
    class PointObstacle {
    public:
        PointObstacle(const Eigen::Vector2d & center, const double & r);

        bool isPointInside(const double px,  const double py);

    private:
        const Eigen::Vector2d center;
        const double r;

    public:
        // exceptions
        struct WrongRadius : public std::exception
        {
            const char* what() const throw() {
                return "PointObstacle: radius must be > 0!";
            }
        };
    };
}


#endif //MRS_LIB_POINTOBSTACLE_H
