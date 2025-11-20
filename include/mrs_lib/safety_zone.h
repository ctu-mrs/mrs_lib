// clang: TomasFormat
#ifndef MRS_LIB_SAFETYZONE_H
#define MRS_LIB_SAFETYZONE_H

#include <mrs_lib/safety_zone/polygon.h>
#include <eigen3/Eigen/Eigen>

namespace mrs_lib
{

namespace safety_zone
{

struct BorderError : public std::exception
{
  const char* what() const throw() {
    return "SafeZone: Wrong configuration for the border";
  }
};

class SafetyZone {
public:
  SafetyZone(std::shared_ptr<Polygon> outerBorder);

  SafetyZone(const Eigen::MatrixXd& outerBorderMatrix);

  bool isPointValid(const double px, const double py);
  bool isPathValid(const double p1x, const double p1y, const double p2x, const double p2y);

  Polygon getBorder();

private:
  std::shared_ptr<Polygon> outerBorder;
};

}  // namespace safety_zone
   //
}  // namespace mrs_lib

#endif  // MRS_LIB_SAFETYZONE_H
