// clang: MatousFormat
#include <mrs_lib/geometry/misc.h>

namespace mrs_lib
{
  namespace geometry
  {

    // instantiation of common template values
    vec_t<3 + 1> toHomogenous(const vec_t<3>& vec);
    vec_t<2 + 1> toHomogenous(const vec_t<2>& vec);

    // | ----------------- Angle-related functions ---------------- |

    /* angle-related functions //{ */

    /* cross() //{ */

    double cross(const vec2_t& vec1, const vec2_t vec2)
    {
      return vec1.x() * vec2.y() - vec1.y() * vec2.x();
    }

    //}

    /* angleBetween() //{ */

    double angleBetween(const vec3_t& vec1, const vec3_t& vec2)
    {
      const double sin_12 = vec1.cross(vec2).norm();
      const double cos_12 = vec1.dot(vec2);
      const double angle = std::atan2(sin_12, cos_12);
      return angle;
    }

    double angleBetween(const vec2_t& vec1, const vec2_t& vec2)
    {
      const double sin_12 = cross(vec1, vec2);
      const double cos_12 = vec1.dot(vec2);
      const double angle = std::atan2(sin_12, cos_12);
      return angle;
    }

    //}

    /* angleaxisBetween() //{ */

    Eigen::AngleAxisd angleaxisBetween(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
    {
      // Find the rotation matrix to rotate vec1 to point in the direction of vec2
      const Eigen::Vector3d a = vec1.normalized();
      const Eigen::Vector3d b = vec2.normalized();
      const Eigen::Vector3d v = a.cross(b);
      const double sin_ab = v.norm();
      const double cos_ab = a.dot(b);
      const double angle = std::atan2(sin_ab, cos_ab);
      Eigen::AngleAxisd ret;
      if (abs(angle) < tolerance)
        ret = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      else if (abs(abs(angle) - M_PI) < tolerance)
        ret = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
      else
        ret = Eigen::AngleAxisd(angle, v.normalized());
      return ret;
    }

    //}

    /* quaternionBetween() //{ */

    Eigen::Quaterniond quaternionBetween(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
    {
      const auto rot = angleaxisBetween(vec1, vec2, tolerance);
      const Eigen::Quaterniond ret(rot);
      return ret;
    }

    //}

    /* rotationBetween() //{ */

    Eigen::Matrix3d rotationBetween(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
    {
      const auto rot = angleaxisBetween(vec1, vec2, tolerance);
      const Eigen::Matrix3d ret(rot);
      return ret;
    }

    //}

    /* haversin() //{ */

    double haversin(const double angle)
    {
      return (1.0 - std::cos(angle)) / 2.0;
    }

    //}

    /* invHaversin() //{ */

    double invHaversin(const double value)
    {
      return 2.0 * std::asin(std::sqrt(value));
    }

    //}

    //}

    /* triangleArea() //{ */

    double triangleArea(const double a, const double b, const double c)
    {
      double s = (a + b + c) / 2.0;
      return std::sqrt(s * (s - a) * (s - b) * (s - c));
    }

    //}

    /* vector distance //{ */

    double dist(const vec2_t& a, const vec2_t& b)
    {

      return (a - b).norm();
    }

    double dist(const vec3_t& a, const vec3_t& b)
    {

      return (a - b).norm();
    }

    //}

    /* quaternionFromEuler() overloads //{ */
    Eigen::Quaterniond quaternionFromEuler(double x, double y, double z)
    {
      return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
    }

    Eigen::Quaterniond quaternionFromEuler(Eigen::Vector3d euler)
    {

      return Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());
    }
    //}

  }  // namespace geometry
}  // namespace mrs_lib
