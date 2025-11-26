// clang: TomasFormat
#ifndef MRS_LIB_POLYGON_H
#define MRS_LIB_POLYGON_H

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/point.hpp>
#include <mrs_lib/safety_zone/line_operations.h>

namespace mrs_lib
{

  namespace safety_zone
  {

    // exceptions
    struct WrongNumberOfVertices : public std::exception
    {
      const char* what() const throw()
      {
        return "Polygon: wrong number of vertices was supplied!";
      }
    };

    struct WrongNumberOfColumns : public std::exception
    {
      const char* what() const throw()
      {
        return "Polygon: wrong number of colums, it should be =2!";
      }
    };

    struct ExtraVertices : public std::exception
    {
      const char* what() const throw()
      {
        return "Polygon: useless vertices detected, polygon methods may break!";
      }
    };

    class Polygon
    {
    public:
      Polygon(const Eigen::MatrixXd vertices);

      bool isPointInside(const double px, const double py);
      bool doesSectionIntersect(const double startX, const double startY, const double endX, const double endY);
      bool isClockwise();
      void inflateSelf(double amount);

      std::vector<geometry_msgs::msg::Point> getPointMessageVector(const double z);

    private:
      Eigen::MatrixXd vertices;
    };

  } // namespace safety_zone

} // namespace mrs_lib

#endif // MRS_LIB_POLYGON_H
