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

namespace mrs_lib
{
  template <int dims>
  using vec_t = Eigen::Matrix<double, dims, 1>;

  using pt2_t = vec_t<2>;
  using vec2_t = vec_t<2>;
  using pt3_t = vec_t<3>;
  using vec3_t = vec_t<3>;

  // branchless, templated, more efficient version of sign
  // taken from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
  template <typename T> int sign(T val)
  {
      return (T(0) < val) - (val < T(0));
  }

  template <int dims>
  vec_t<dims+1> to_homogenous(const vec_t<dims>& vec)
  {
    const Eigen::Matrix<double, dims+1, 1> ret( (Eigen::Matrix<double, dims+1, 1>() << vec, 1).finished() );
    return ret;
  }

  /* Rotation and angle related functions //{ */

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

  double haversin(double angle);
  double invHaversin(double angle);

  double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);
  double solidAngle(double a, double b, double c);
  double rectSolidAngle(Rectangle r, Eigen::Vector3d center);

  double triangleArea(double a, double b, double c);
  double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

}  // namespace mrs_lib

#endif
