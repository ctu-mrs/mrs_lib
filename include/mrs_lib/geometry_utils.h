// clang: MatousFormat
/**  \file
     \brief Defines useful geometry utilities and functions.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <iostream>

namespace mrs_lib
{
  template <int dims>
  using vec_t = Eigen::Matrix<double, dims, 1>;

  using pt2_t = vec_t<2>;
  using vec2_t = vec_t<2>;
  using pt3_t = vec_t<3>;
  using vec3_t = vec_t<3>;

  template <int dims>
  vec_t<dims + 1> to_homogenous(const vec_t<dims>& vec)
  {
    const Eigen::Matrix<double, dims + 1, 1> ret((Eigen::Matrix<double, dims + 1, 1>() << vec, 1).finished());
    return ret;
  }

  /* Rotation and angle related functions //{ */

  /*!
   * \brief Normalizes the \p angle to the range \f$ < \mathrm{from}, \mathrm{from} + 2\pi > \f$.
   *
   * \param angle the angle to be normalized.
   * \param from  start of the interval into which the angle shall be normalized.
   *
   * \returns     the normalized angle.
   *
   */
  double normalize_angle(const double angle, const double from = -M_PI);

  /*!
   * \brief Normalizes the \p angle to the range \f$ < \mathrm{from}, \mathrm{to} > \f$.
   *
   * \param angle the angle to be normalized.
   * \param from  start of the interval into which the angle shall be normalized.
   * \param to    end of the interval into which the angle shall be normalized.
   *
   * \returns     the normalized angle.
   *
   */
  double normalize_angle(const double angle, const double from, const double to);

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
  double angle_between(const vec2_t& a, const vec2_t& b);

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
  double angle_between(const vec3_t& a, const vec3_t& b);

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
  Eigen::AngleAxisd angleaxis_between(const vec3_t& a, const vec3_t& b, const double tolerance = 1e-9);

  /*!
   * \brief Returns the rotation between two vectors, represented as a quaternion.
   *
   * Works the same as the angleaxis_between() function (in fact it is used in the implementation).
   *
   * \param a vector from which the rotation starts.
   * \param b vector at which the rotation ends.
   *
   * \returns    rotation from \p a to \p b.
   *
   */
  Eigen::Quaterniond quaternion_between(const vec3_t& a, const vec3_t& b, const double tolerance = 1e-9);

  /*!
   * \brief Returns the rotation between two vectors, represented as a rotation matrix.
   *
   * Works the same as the angleaxis_between() function (in fact it is used in the implementation).
   *
   * \param a vector from which the rotation starts.
   * \param b vector at which the rotation ends.
   *
   * \returns    rotation from \p a to \p b.
   *
   */
  Eigen::Matrix3d rotation_between(const vec3_t& a, const vec3_t& b, const double tolerance = 1e-9);

  //}

  /* UNUSED //{ */

  /*!
   * \brief A class, representing a conic section (ellipse, hyperbola, parabola etc.).
   *
   * For the quadratic representation, the notation \f$ Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0 \f$ is used.
   * Related online information:
   * * https://en.wikipedia.org/wiki/Conic_section#Matrix_notation
   * * https://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections
   * * https://en.wikipedia.org/wiki/Ellipse
   * * https://en.wikipedia.org/wiki/Hyperbola
   * * http://mathworld.wolfram.com/ConicSection.html
   * * http://mathworld.wolfram.com/Ellipse.html
   * * http://mathworld.wolfram.com/Hyperbola.html
   *
   */
  /* class ConicSection */
  /* { */
  /*   public: */
  /*     enum conic_type_t */
  /*     { */
  /*       // helper values */
  /*       degenerate = 0x10, */
  /*       special_case = 0x20, */
  /*       basic_type_mask = 0x0F, */
  /*       // non-degenerate types */
  /*       ellipse = 0, */
  /*       parabola = 1, */
  /*       hyperbola = 2, */
  /*       // special cases */
  /*       circle = 0 | special_case, */
  /*       rectangular_hyperbola = 2 | special_case, */
  /*       // degenerate types */
  /*       point = 0 | degenerate, */
  /*       parallel_lines = 1 | degenerate, */
  /*       intersecting_lines = 2 | degenerate, */
  /*     }; */

  /*     struct central_conic_params_t */
  /*     { */
  /*       double a; */
  /*       double b; */
  /*       double x0; */
  /*       double y0; */
  /*       double theta; */
  /*     }; */

  /*   public: */
  /*     ConicSection(const double A, const double B, const double C, const double D, const double E, const double F, const double tolerance = 1e-9); */

  /*     static conic_type_t classify_conic_section(const double alpha, const double beta, const double tolerance = 1e-9); */

  /*     static double closest_point_on_ellipse_param(const pt2_t& to_point, const central_conic_params_t& ellipse_params); */

  /*   private: */
  /*     double A, B, C, D, E, F; */
  /*     double alpha, beta; */
  /*     conic_type_t conic_type; */

  /*     central_conic_params_t calc_central_conic_params(); */
  /* }; */

  //}

  /* class Ray //{ */

  class Ray
  {
  public:
    Ray();
    ~Ray();
    Ray(Eigen::Vector3d origin, Eigen::Vector3d direction, double energy = 0.0);

    double energy;
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;

    static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo)
    {
      Eigen::Vector3d origin = pointFrom;
      Eigen::Vector3d direction = (pointTo - pointFrom);
      /* direction.normalize(); */
      return Ray(origin, direction);
    }
    static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction)
    {
      return Ray(origin, direction);
    }
  };

  //}

  /* class Plane //{ */

  class Plane
  {
  public:
    Plane();
    ~Plane();
    Plane(Eigen::Vector3d point, Eigen::Vector3d normal);

    Eigen::Vector3d point;
    Eigen::Vector3d normal;

    std::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
  };

  //}

  /* class Rectangle //{ */

  class Rectangle
  {
  public:
    Rectangle();
    ~Rectangle();
    Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);

    Eigen::Vector3d normal_vector;

    Eigen::Matrix3d basis;
    Eigen::Matrix3d projector;

    Plane plane;
    std::vector<Eigen::Vector3d> points;

    std::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
  };

  //}

  /* class Cone //{ */

  class Cone
  {

  public:
    Cone(Eigen::Vector3d position, Eigen::Vector3d direction, double angle);
    ~Cone();
    Eigen::Vector3d ProjectPoint(Eigen::Vector3d point);

    /* NOT FINISHED //{ */
    std::optional<Eigen::Vector3d> ProjectPointOnPlane(Plane plane, Eigen::Vector3d point);
    //}

  private:
    Eigen::Vector3d position, direction;
    double angle;

    Eigen::MatrixXd cone_axis_projector;
    Ray cone_ray;
  };

  //}

  // | ----------------------- conversions ---------------------- |

  /* class EulerAttitude //{ */

  class EulerAttitude
  {
  public:
    EulerAttitude(const double& r, const double& p, const double& y) : r(r), p(p), y(y){};

    double roll(void) const
    {
      return r;
    }

    double pitch(void) const
    {
      return p;
    }

    double yaw(void) const
    {
      return y;
    }

  private:
    double r, p, y;
  };

  //}

  /* class AttitudeConvertor //{ */

  class AttitudeConvertor
  {
  public:
    AttitudeConvertor(const double& roll, const double& pitch, const double& yaw)
    {
      tf2_quaternion.setRPY(roll, pitch, yaw);
    }

    AttitudeConvertor(const tf::Quaternion quaternion)
    {
      tf2_quaternion.setX(quaternion.x());
      tf2_quaternion.setY(quaternion.y());
      tf2_quaternion.setZ(quaternion.z());
      tf2_quaternion.setW(quaternion.w());
    }

    AttitudeConvertor(const geometry_msgs::Quaternion quaternion)
    {
      tf2_quaternion.setX(quaternion.x);
      tf2_quaternion.setY(quaternion.y);
      tf2_quaternion.setZ(quaternion.z);
      tf2_quaternion.setW(quaternion.w);
    }

    AttitudeConvertor(const mrs_lib::EulerAttitude& euler_attitude)
    {
      tf2_quaternion.setRPY(euler_attitude.roll(), euler_attitude.pitch(), euler_attitude.yaw());
    }

    AttitudeConvertor(const Eigen::Quaterniond quaternion)
    {
      tf2_quaternion.setX(quaternion.x());
      tf2_quaternion.setY(quaternion.y());
      tf2_quaternion.setZ(quaternion.z());
      tf2_quaternion.setW(quaternion.w());
    }

    AttitudeConvertor(const Eigen::Matrix3d matrix)
    {
      Eigen::Quaterniond quaternion = Eigen::Quaterniond(matrix);

      tf2_quaternion.setX(quaternion.x());
      tf2_quaternion.setY(quaternion.y());
      tf2_quaternion.setZ(quaternion.z());
      tf2_quaternion.setW(quaternion.w());
    }

    template <class T>
    AttitudeConvertor(const Eigen::AngleAxis<T> angle_axis)
    {
      double angle = angle_axis.angle();
      tf2::Vector3 axis(angle_axis.axis()[0], angle_axis.axis()[1], angle_axis.axis()[2]);

      tf2_quaternion.setRotation(axis, angle);
    }

    AttitudeConvertor(const tf2::Quaternion quaternion)
    {

      tf2_quaternion = quaternion;
    }

    operator tf2::Transform() const
    {
      return tf2::Transform(tf2_quaternion);
    }

    operator tf2::Quaternion() const
    {
      return tf2_quaternion;
    }

    operator tf::Quaternion() const
    {
      tf::Quaternion tf_quaternion;

      tf_quaternion.setX(tf2_quaternion.x());
      tf_quaternion.setY(tf2_quaternion.y());
      tf_quaternion.setZ(tf2_quaternion.z());
      tf_quaternion.setW(tf2_quaternion.w());

      return tf_quaternion;
    }

    operator geometry_msgs::Quaternion() const
    {
      geometry_msgs::Quaternion geom_quaternion;

      geom_quaternion.x = tf2_quaternion.x();
      geom_quaternion.y = tf2_quaternion.y();
      geom_quaternion.z = tf2_quaternion.z();
      geom_quaternion.w = tf2_quaternion.w();

      return geom_quaternion;
    }

    operator EulerAttitude() const
    {

      double roll, pitch, yaw;
      tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw);

      return EulerAttitude(roll, pitch, yaw);
    }

    template <class T>
    operator Eigen::AngleAxis<T>() const
    {

      double angle = tf2_quaternion.getAngle();
      Eigen::Vector3d axis(tf2_quaternion.getAxis()[0], tf2_quaternion.getAxis()[1], tf2_quaternion.getAxis()[2]);

      Eigen::AngleAxis<T> angle_axis(angle, axis);

      return angle_axis;
    }

    template <class T>
    operator Eigen::Quaternion<T>() const
    {

      return Eigen::Quaternion<T>(tf2_quaternion.w(), tf2_quaternion.x(), tf2_quaternion.y(), tf2_quaternion.z());
    }

    operator Eigen::Matrix3d() const
    {
      Eigen::Quaterniond quaternion(tf2_quaternion.w(), tf2_quaternion.x(), tf2_quaternion.y(), tf2_quaternion.z());

      return quaternion.toRotationMatrix();
    }

    std::tuple<double, double, double> getRPY()
    {
      double roll, pitch, yaw;
      tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw);

      return std::tuple(roll, pitch, yaw);
    }

    double getYaw()
    {
      double roll, pitch, yaw;
      tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw);

      return yaw;
    }

    /* double getRoll() */
    /* { */
    /*   double roll, pitch, yaw; */
    /*   tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw); */

    /*   return roll; */
    /* } */

    /* double getPitch() */
    /* { */
    /*   double roll, pitch, yaw; */
    /*   tf2::Matrix3x3(tf2_quaternion).getRPY(roll, pitch, yaw); */

    /*   return pitch; */
    /* } */

  private:
    tf2::Quaternion tf2_quaternion;
  };

  //}

  /**
   * @brief calculate the distance between two points in 2d
   *
   * @param ax
   * @param ay
   * @param bx
   * @param by
   *
   * @return the distance
   */
  double dist2d(const double& ax, const double& ay, const double& bx, const double& by);

  /**
   * @brief calculate the distance between two points in 3d
   *
   * @param ax
   * @param ay
   * @param az
   * @param bx
   * @param by
   * @param
   *
   * @return
   */
  double dist3d(const double& ax, const double& ay, const double& az, const double& bx, const double& by, const double& bz);

  double haversin(const double& angle);
  double invHaversin(const double& angle);

  double vectorAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
  double solidAngle(const double& a, const double& b, const double& c);
  double rectSolidAngle(const Rectangle& r, const Eigen::Vector3d& center);

  double triangleArea(const double& a, const double& b, const double& c);
  double sphericalTriangleArea(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);

  double unwrapAngle(const double& yaw, const double& yaw_previous);
  double wrapAngle(const double& angle_in);
  double disambiguateAngle(const double& yaw, const double& yaw_previous);
  double angleBetween(const double& a, const double& b);
  double interpolateAngles(const double& a1, const double& a2, const double& coeff);
  Eigen::Vector2d rotateVector2d(const Eigen::Vector2d& vector_in, const double& angle);

}  // namespace mrs_lib

#endif
