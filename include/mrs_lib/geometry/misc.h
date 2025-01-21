// clang: MatousFormat
/**  \file
     \brief Defines useful geometry utilities and functions.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Petr Štibinger - stibipet@fel.cvut.cz
 */

#ifndef GEOMETRY_MISC_H
#define GEOMETRY_MISC_H

#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/quaternion.h>

namespace mrs_lib
{
  namespace geometry
  {
    template <int dims>
    using vec_t = Eigen::Matrix<double, dims, 1>;

    using pt2_t = vec_t<2>;
    using vec2_t = vec_t<2>;
    using pt3_t = vec_t<3>;
    using vec3_t = vec_t<3>;

    using mat3_t = Eigen::Matrix3d;

    using quat_t = Eigen::Quaterniond;
    using anax_t = Eigen::AngleAxisd;

    template <int dims>
    vec_t<dims + 1> toHomogenous(const vec_t<dims>& vec)
    {
      const Eigen::Matrix<double, dims + 1, 1> ret((Eigen::Matrix<double, dims + 1, 1>() << vec, 1).finished());
      return ret;
    }

    // | ----------------- Angle-related functions ---------------- |

    /* angle-related functions //{ */

    /* headingFromRot() //{ */

    /*!
     * \brief Returns the heading angle from the rotation.
     *
     * Heading is defined as the angle of a [1,0,0] vector rotated by the rotation and projected to the XY plane from the X axis.
     *
     * \param rot the rotation to extract the heading angle from.
     *
     * \returns the heading angle.
     *
     */
    template <typename T>
    double headingFromRot(const T& rot)
    {
      const vec3_t rot_vec = rot * vec3_t::UnitX();
      return std::atan2(rot_vec.y(), rot_vec.x());
    }

    //}

    /* angleBetween() //{ */

    /*!
     * \brief Returns the angle between two vectors, taking orientation into account.
     *
     * This implementation uses \p atan2 instead of just \p acos and thus it properly
     * takes into account orientation of the vectors, returning angle in all four quadrants.
     *
     * \param a vector from which the angle will be measured.
     * \param b vector to which the angle will be measured.
     *
     * \returns    angle from \p a to \p b.
     *
     */
    double angleBetween(const vec2_t& a, const vec2_t& b);

    /*!
     * \brief Returns the angle between two vectors, taking orientation into account.
     *
     * This implementation uses \p atan2 instead of just \p acos and thus it properly
     * takes into account orientation of the vectors, returning angle in all four quadrants.
     *
     * \param a vector from which the angle will be measured.
     * \param b vector to which the angle will be measured.
     *
     * \returns    angle from \p a to \p b.
     *
     */
    double angleBetween(const vec3_t& a, const vec3_t& b);

    //}

    /* angleaxisBetween() //{ */

    /*!
     * \brief Returns the rotation between two vectors, represented as angle-axis.
     *
     * To avoid singularities, a \p tolerance parameter is used:
     * * If the absolute angle between the two vectors is less than \p tolerance, a zero rotation is returned.
     * * If the angle between the two vectors is closer to \f$ \pi \f$ than \p tolerance, a \f$ \pi \f$ rotation is returned.
     *
     * \param a vector from which the rotation starts.
     * \param b vector at which the rotation ends.
     *
     * \returns    rotation from \p a to \p b.
     *
     */
    anax_t angleaxisBetween(const vec3_t& a, const vec3_t& b, const double tolerance = 1e-9);

    //}

    /* quaternionBetween() //{ */

    /*!
     * \brief Returns the rotation between two vectors, represented as a quaternion.
     *
     * Works the same as the angleaxisBetween() function (in fact it is used in the implementation).
     *
     * \param a vector from which the rotation starts.
     * \param b vector at which the rotation ends.
     *
     * \returns    rotation from \p a to \p b.
     *
     */
    quat_t quaternionBetween(const vec3_t& a, const vec3_t& b, const double tolerance = 1e-9);

    //}

    /* rotationBetween() //{ */

    /*!
     * \brief Returns the rotation between two vectors, represented as a rotation matrix.
     *
     * Works the same as the angleaxisBetween() function (in fact it is used in the implementation).
     *
     * \param a vector from which the rotation starts.
     * \param b vector at which the rotation ends.
     *
     * \returns    rotation from \p a to \p b.
     *
     */
    Eigen::Matrix3d rotationBetween(const vec3_t& a, const vec3_t& b, const double tolerance = 1e-9);

    //}

    /* haversin() //{ */

    /**
     * @brief computes the haversine (half of versine) for a given angle
     *
     * @param angle angle in radians
     *
     * @return
     */
    double haversin(const double angle);

    //}

    /* invHaversin() //{ */

    /**
     * @brief computes the inverse haversine angle for a given value
     *
     * @param value
     *
     * @return angle in radians
     */
    double invHaversin(const double value);

    //}

    /* solidAngle //{ */

    /**
     * @brief computes the solid angle for a spherical surface corresponding to a 'triangle' with edge lengths a, b, c
     *
     * @param a
     * @param b
     * @param c
     *
     * @return solid angle in steradians
     */
    double solidAngle(double a, double b, double c);

    //}

    /* quaternionFromEuler() //{ */

    /**
     * @brief create a quaternion from 3 provided Euler angles
     *
     * @param x Euler angle in radians
     * @param y Euler angle in radians
     * @param z Euler angle in radians
     *
     * @return quaternion
     */
    quat_t quaternionFromEuler(double x, double y, double z);

    /**
     * @brief create a quaternion from Euler angles provided as a vector
     *
     * @param euler components of the rotation provided as vector of Euler angles
     *
     * @return quaternion
     */
    quat_t quaternionFromEuler(const Eigen::Vector3d& euler);

    //}

    /**
     * @brief create a quaternion from heading
     *
     * Heading is defined as the angle of a [1,0,0] vector rotated by the rotation and projected to the XY plane from the X axis.
     *
     * @param heading the heading angle.
     *
     * @return the quaternion corresponding to the heading rotation.
     */
    quat_t quaternionFromHeading(const double heading);

    //}

    // | ----------------- Miscellaneous functions ---------------- |

    /* 2D cross() //{ */

    /*!
     * \brief Implementation of cross product for 2D vectors.
     *
     * Useful e.g. for finding the sine of an angle between two 2D vectors.
     *
     * \param a first vector of the cross product.
     * \param b second vector of the cross product.
     *
     * \returns    \f$ a \times b \f$ (sine of the angle from \p a to \p b).
     *
     */
    double cross(const vec2_t& a, const vec2_t b);

    //}

    /* vector distance //{ */

    /**
     * @brief distnace between two 2D Eigen vectors
     *
     * @param a
     * @param b
     *
     * @return Euclidean distance
     */
    double dist(const vec2_t& a, const vec2_t& b);

    /**
     * @brief distnace between two 3D Eigen vectors
     *
     * @param a
     * @param b
     *
     * @return Euclidean distance
     */
    double dist(const vec3_t& a, const vec3_t& b);

    //}

    /* triangleArea() //{ */

    /**
     * @brief uses Heron's formula to compute area of a given triangle using side lengths
     *
     * @param a length of side1
     * @param b length of side2
     * @param c length of side3
     *
     * @return area in units squared
     */
    double triangleArea(const double a, const double b, const double c);

    //}

    /* sphericalTriangleArea //{ */

    /**
     * @brief compute the area of a 'triangle' drawn on a spherical surface
     *
     * @param a length of side1
     * @param b length of side2
     * @param c length of side3
     *
     * @return area in units squared
     */
    double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

    //}

    /* rotateCovariance() //{ */

    /*!
     * \brief Returns the covariance rotated using the specified rotation.
     *
     * \param cov the covariance to be rotated.
     * \param rot the rotation use.
     *
     * \returns a new matrix object containing the rotated covariance.
     *
     */
    template <typename T>
    mat3_t rotateCovariance(const mat3_t& cov, const T& rot)
    {
      const mat3_t matrot(rot);
      return matrot * cov * matrot.transpose();
    }

    //}

  }  // namespace geometry
}  // namespace mrs_lib

#endif
