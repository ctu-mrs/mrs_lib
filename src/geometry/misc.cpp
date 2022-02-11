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

    anax_t angleaxisBetween(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
    {
      // Find the rotation matrix to rotate vec1 to point in the direction of vec2
      const Eigen::Vector3d a = vec1.normalized();
      const Eigen::Vector3d b = vec2.normalized();
      const Eigen::Vector3d v = a.cross(b);
      const double sin_ab = v.norm();
      const double cos_ab = a.dot(b);
      const double angle = std::atan2(sin_ab, cos_ab);
      anax_t ret;
      if (abs(angle) < tolerance)
        ret = anax_t(0.0, Eigen::Vector3d::UnitX());
      else if (abs(abs(angle) - M_PI) < tolerance)
        ret = anax_t(M_PI, Eigen::Vector3d::UnitX());
      else
        ret = anax_t(angle, v.normalized());
      return ret;
    }

    //}

    /* quaternionBetween() //{ */

    quat_t quaternionBetween(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
    {
      const auto rot = angleaxisBetween(vec1, vec2, tolerance);
      const quat_t ret(rot);
      return ret;
    }

    /* quaternionFromEuler() overloads //{ */
    quat_t quaternionFromEuler(double x, double y, double z)
    {
      return anax_t(x, vec3_t::UnitX()) * anax_t(y, vec3_t::UnitY()) * anax_t(z, vec3_t::UnitZ());
    }

    quat_t quaternionFromEuler(const Eigen::Vector3d& euler)
    {
      return anax_t(euler.x(), vec3_t::UnitX()) * anax_t(euler.y(), vec3_t::UnitY())
             * anax_t(euler.z(), vec3_t::UnitZ());
    }
    //}

    /* quaternionFromHeading //{ */
    quat_t quaternionFromHeading(const double heading)
    {
      return quat_t(anax_t(heading, Eigen::Vector3d::UnitZ()));
    }
    //}

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

    /* solidAngle() //{ */
    double solidAngle(double a, double b, double c)
    {
      return invHaversin((haversin(c) - haversin(a - b)) / (std::sin(a) * std::sin(b)));
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

    /* sphericalTriangleArea //{ */
    double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
    {
      double ab = angleBetween(a, b);
      double bc = angleBetween(b, c);
      double ca = angleBetween(c, a);

      if (ab < 1e-3 and bc < 1e-3 and ca < 1e-3)
      {
        return triangleArea(ab, bc, ca);
      }

      double A = solidAngle(ca, ab, bc);
      double B = solidAngle(ab, bc, ca);
      double C = solidAngle(bc, ca, ab);

      return A + B + C - M_PI;
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

  }  // namespace geometry
}  // namespace mrs_lib
