// clang: MatousFormat
/**  \file
     \brief Defines useful geometry utilities and functions.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Petr Štibinger - stibipet@fel.cvut.cz
 */

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <vector>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>

#include <mrs_lib/utils.h>

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

  /* class Ray //{ */

  /**
   * @brief geometric representation of a ray. Instantiate it by two input Vector3. Use static methods for from-to raycast, or a point-direction raycast.
   */
  class Ray
  {
  public:
    /**
     * @brief constructor without initialization of internal variables
     */
    Ray();

    /**
     * @brief destructor
     */
    ~Ray();

    /**
     * @brief default constructor
     *
     * @param p1 origin of the ray
     * @param p2 endpoint of the ray
     */
    Ray(Eigen::Vector3d p1, Eigen::Vector3d p2);

  private:
    Eigen::Vector3d point1;
    Eigen::Vector3d point2;

  public:
    /**
     * @brief get the origin point
     *
     * @return ray origin point
     */
    const Eigen::Vector3d p1();
    /**
     * @brief get the end point
     *
     * @return ray end point
     */
    const Eigen::Vector3d p2();

    /**
     * @brief get the direction of ray (normalized)
     *
     * @return direction (normalized)
     */
    const Eigen::Vector3d direction();

  public:
    /**
     * @brief static method for generating new rays by raycasting from-to
     *
     * @param pointFrom origin of the ray
     * @param pointTo endpoint of the ray
     *
     * @return new Ray instance created by raycasting
     */
    static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo);

    /**
     * @brief static method for generating new rays by raycasting origin-direction
     *
     * @param origin origin of the ray
     * @param direction of the ray
     *
     * @return new Ray instance created by raycasting
     */
    static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction);
  };

  //}

  /* class Triangle //{ */
  /**
   * @brief geometric representation of a triangle. Instantiate a new triangle by providing three vertices
   */
  class Triangle
  {
  public:
    /**
     * @brief constructor for initialization with default values (points [0,0,0], [1,0,0] and [0,0,1]
     */
    Triangle();

    /**
     * @brief destructor
     */
    ~Triangle();

    /**
     * @brief default constructor for creating a new triangle from given vertices
     *
     * @param a
     * @param b
     * @param c
     */
    Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

  private:
    Eigen::Vector3d point1;
    Eigen::Vector3d point2;
    Eigen::Vector3d point3;

  public:
    const Eigen::Vector3d a();
    const Eigen::Vector3d b();
    const Eigen::Vector3d c();

    /**
     * @brief get position on the triangle center
     *
     * @return vector3
     */
    const Eigen::Vector3d center();

    /**
     * @brief get normal vector of this triangle. The vector origin is placed at triangle center, length is normalized and direction follows the right-hand rule
     * with respect to vertex order a-b-c
     *
     * @return vector3
     */
    const Eigen::Vector3d normal();

    /**
     * @brief get a vector of all vertices
     *
     * @return std::vector<vector3>
     */
    const std::vector<Eigen::Vector3d> vertices();

  public:
    /**
     * @brief calculate an intersection of this triangle with a given ray with given tolerance
     *
     * @param r ray to calculate intersection with
     * @param epsilon calculation tolerance
     *
     * @return vector3 intersection if exists, boost::none if no intersection is found
     */
    const boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4);
  };
  //}

  /* class Rectangle //{ */

  /**
   * @brief geometric representation of a rectangle (can represent any quadrilateral)
   */
  class Rectangle
  {
  public:
    /**
     * @brief constructor for initialization with points set to [0,0,0], [1,0,0], [1,1,0] and [0,1,0]
     */
    Rectangle();

    /**
     * @brief destructor
     */
    ~Rectangle();

    /**
     * @brief constructor using a std::vector of points (vector3). Provide points in a counter-clockwise order for correct behavior
     *
     * @param points std::vector of points in 3d
     */
    Rectangle(std::vector<Eigen::Vector3d> points);

    /**
     * @brief constructor using four points (vector3). Provide points in a counter-clockwise order for correct behavior
     *
     * @param a
     * @param b
     * @param c
     * @param d
     */
    Rectangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d);

  private:
    Eigen::Vector3d point1;
    Eigen::Vector3d point2;
    Eigen::Vector3d point3;
    Eigen::Vector3d point4;

  public:
    /**
     * @brief getter for first point
     *
     * @return 1st point (vector3)
     */
    const Eigen::Vector3d a();

    /**
     * @brief getter for the second point
     *
     * @return 2nd point (vector3)
     */
    const Eigen::Vector3d b();

    /**
     * @brief getter for the third point
     *
     * @return 3rd point (vector3)
     */
    const Eigen::Vector3d c();

    /**
     * @brief getter for the fourth point
     *
     * @return 4th point (vector3)
     */
    const Eigen::Vector3d d();

    /**
     * @brief getter for center point
     *
     * @return center point (vector3)
     */
    const Eigen::Vector3d center();

    /**
     * @brief getter for the normal vector. It originates in the center of the Rectangle, length is normalized, orientation follows the right-hand rule,
     * assuiming the points are provided in counter-clockwise order
     *
     * @return normal vector3
     */
    const Eigen::Vector3d normal();

    /**
     * @brief getter for all the points of this rectangle provided as std::vector
     *
     * @return std::vector of points (vector3)
     */
    const std::vector<Eigen::Vector3d> vertices();

    /**
     * @brief getter for the triangles forming this rectangle
     *
     * @return std::vector of triangles
     */
    const std::vector<Triangle> triangles();

    /**
     * @brief calculate an intersection of this rectangle with a given ray with given tolerance
     *
     * @param r ray to calculate intersection with
     * @param epsilon calculation tolerance
     *
     * @return vector3 intersection if exists, boost::none if no intersection is found
     */
    const boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4);

    /**
     * @brief check if the normal is facing a given point, i.e. if the point lies in the same half-space as the rectangle normal
     *
     * @param point vector3 to check against
     *
     * @return true if the normal is facing given point. Returns false for angle >= 90 degrees
     */
    bool isFacing(Eigen::Vector3d point);
  };

  //}

  /* class Cuboid //{ */
  /**
   * @brief geometric representation of a cuboid
   */
  class Cuboid
  {
  public:
    /**
     * @brief constructor for initialization with all points set to [0,0,0]
     */
    Cuboid();

    /**
     * @brief virtual destructor
     */
    ~Cuboid();

    /**
     * @brief constructor from six provided points (vector3)
     *
     * @param p0 vector3
     * @param p1 vector3
     * @param p2 vector3
     * @param p3 vector3
     * @param p4 vector3
     * @param p5 vector3
     * @param p6 vector3
     * @param p7 vector3
     */
    Cuboid(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4, Eigen::Vector3d p5, Eigen::Vector3d p6,
           Eigen::Vector3d p7);
    /**
     * @brief constructor from a std::vector of provided points
     *
     * @param points std::vector<vector3>
     */
    Cuboid(std::vector<Eigen::Vector3d> points);

    /**
     * @brief constructor from a provided center point, size and orientation
     *
     * @param center vector3
     * @param size vector3
     * @param orientation quaternion
     */
    Cuboid(Eigen::Vector3d center, Eigen::Vector3d size, Eigen::Quaterniond orientation);

  private:
    /**
     * @brief side indexing
     */
    enum
    {
      FRONT = 0,
      BACK = 1,
      LEFT = 2,
      RIGHT = 3,
      BOTTOM = 4,
      TOP = 5,
    };

  private:
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> lookupPoints(int face_idx);

  public:
    /**
     * @brief getter for all vertices (vector3) of this cuboid
     *
     * @return std::vector<vector3>
     */
    const std::vector<Eigen::Vector3d> vertices();

    /**
     * @brief getter for the center point
     *
     * @return vector3
     */
    const Eigen::Vector3d center();

    /**
     * @brief getter for one side corresponding to a provided index
     *
     * @param face_idx index of the side to lookup points for
     *
     * @return rectangle representing the cuboid side
     */
    const Rectangle getRectangle(int face_idx);

    /**
     * @brief calculate the intersection between this cuboid and a provided ray within a given tolerance. Can result in 0, 1 or 2 intersection points
     *
     * @param r ray to check intersection with
     * @param epsilon tolerance for the calculation
     *
     * @return std::vector<vector3> list of intersection points (depending on the geometry, can yield 0, 1 or 2 points)
     */
    const std::vector<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4);
  };
  //}

  /* class Ellipse //{ */
  /**
   * @brief geometric representation of an ellipse
   */
  class Ellipse
  {
  public:
    /**
     * @brief constructor for initialization without setting internal variables
     */
    Ellipse();

    /**
     * @brief destructor
     */
    ~Ellipse();

    /**
     * @brief constructor using a provided center point, orientation, major semi-axis length and minor semi-axis length
     *
     * @param center vector3
     * @param orientation quaternion
     * @param a major semi-axis length
     * @param b minor semi-axis length
     */
    Ellipse(Eigen::Vector3d center, Eigen::Quaterniond orientation, double a, double b);

  private:
    double major_semi;
    double minor_semi;
    Eigen::Vector3d center_point;
    Eigen::Quaterniond absolute_orientation;

  public:
    /**
     * @brief getter for major semi-axis
     *
     * @return length of major semi-axis
     */
    double a();

    /**
     * @brief getter for minor semi-axis
     *
     * @return length of minor semi-axis
     */
    double b();

    /**
     * @brief getter for the center point
     *
     * @return vector3
     */
    const Eigen::Vector3d center();

    /**
     * @brief getter for the orientation
     *
     * @return quaternion
     */
    const Eigen::Quaterniond orientation();
  };
  //}

  /* class Cylinder //{ */
  /**
   * @brief geometric representation of a cylinder
   */
  class Cylinder
  {
  public:
    /**
     * @brief constructor without setting the internal variables
     */
    Cylinder();

    /**
     * @brief destructor
     */
    ~Cylinder();

    /**
     * @brief constructor using a given center point, radius, height and orietnation
     *
     * @param center geometric center in the middle of body height
     * @param radius radius of the cap
     * @param height distance between caps
     * @param orientation quaternion
     */
    Cylinder(Eigen::Vector3d center, double radius, double height, Eigen::Quaterniond orientation);

  private:
    Eigen::Vector3d center_point;
    double radius;
    double height;
    Eigen::Quaterniond absolute_orientation;

  public:
    /**
     * @brief cap indexing
     */
    enum
    {
      BOTTOM = 0,
      TOP = 1,
    };

  public:
    /**
     * @brief getter for the center point
     *
     * @return vector3
     */
    const Eigen::Vector3d center();

    /**
     * @brief getter for the orientation
     *
     * @return quaternion
     */
    const Eigen::Quaterniond orientation();

    /**
     * @brief getter for cap radius
     *
     * @return radius of the cap
     */
    double r();

    /**
     * @brief getter for the body height
     *
     * @return body height
     */
    double h();

    /**
     * @brief getter for a cap corresponding to a provided index
     *
     * @param index of the cap
     *
     * @return ellipse representing the cap
     */
    const Ellipse getCap(int index);
  };
  //}

  /* class Cone //{ */
  /**
   * @brief geometric representaiton of a cone
   */
  class Cone
  {
  public:
    /**
     * @brief constructor without setting the internal variables
     */
    Cone();

    /**
     * @brief destructor
     */
    ~Cone();

    /**
     * @brief constructor from a given origin point (tip of the cone), angle, height and orientation
     *
     * @param origin_point tip of the cone
     * @param angle angle between body height and side in radians
     * @param height distance between tip and base
     * @param orientation offset from the default orientation. Default orientation is with body height aligned with Z axis, standing on the tip
     */
    Cone(Eigen::Vector3d origin_point, double angle, double height, Eigen::Vector3d orientation);

  private:
    Eigen::Vector3d origin_point;
    double angle;
    double height;
    Eigen::Vector3d absolute_direction;

  public:
    /**
     * @brief getter for the tip point
     *
     * @return vector3
     */
    const Eigen::Vector3d origin();

    /**
     * @brief getter for the direction. Normalized direction from origin towards base
     *
     * @return vector3, normalized
     */
    const Eigen::Vector3d direction();

    /**
     * @brief getter for the center point. Center point lies in half of the body height
     *
     * @return vector3
     */
    const Eigen::Vector3d center();

    /**
     * @brief getter for angle between body height and body side
     *
     * @return angle in radians
     */
    double theta();

    /**
     * @brief getter for body height
     *
     * @return
     */
    double h();

    /**
     * @brief getter for the cap of the cone
     *
     * @return ellipse representing the cap of the cone
     */
    const Ellipse getCap();

    /**
     * @brief Project a 3D point orthogonally onto the Cone surface
     *
     * @param point in 3D
     *
     * @return projected point
     */
    const std::optional<Eigen::Vector3d> projectPoint(const Eigen::Vector3d& point);
  };
  //}

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

  /**
   * @brief computes the haversine (half of versine) for a given angle
   *
   * @param angle angle in radians
   *
   * @return
   */
  double haversin(const double& angle);

  /**
   * @brief computes the inverse haversine angle for a given value
   *
   * @param value
   *
   * @return angle in radians
   */
  double invHaversin(const double& value);

  /**
   * @brief compute the angle between two vectors
   *
   * @param v1 vector3 at which the rotation starts
   * @param v2 vector3 at which the rotation ends
   *
   * @return angle in radians
   */
  double vectorAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

  /**
   * @brief overloaded version to compute the angle between two integer coordinates
   *
   * @param v1 vector2i at which the rotation starts
   * @param v2 vector2i at which the rotation ends
   *
   * @return angle in radians
   */
  double vectorAngle(const Eigen::Vector2i& v1, const Eigen::Vector2i& v2);

  double solidAngle(const double& a, const double& b, const double& c);

  double unwrapAngle(const double& angle, const double& angle_previous);

  double wrapAngle(const double& angle);

  double disambiguateAngle(const double& angle, const double& angle_previous);

  double angleBetween(const double& a, const double& b);

  double interpolateAngles(const double& a1, const double& a2, const double& coeff);

  /**
   * @brief compute the solid angle of a rectangle when viewed from a given point
   *
   * @param r rectangle
   * @param center origin of the solid angle
   *
   * @return solid angle in steradians
   */
  double rectSolidAngle(Rectangle r, Eigen::Vector3d center);


  /**
   * @brief create a quaternion from 3 provided Euler angles
   *
   * @param x Euler angle in radians
   * @param y Euler angle in radians
   * @param z Euler angle in radians
   *
   * @return quaternion
   */
  Eigen::Quaterniond quaternionFromEuler(double x, double y, double z);

  /**
   * @brief create a quaternion from Euler angles provided as a vector
   *
   * @param euler components of the rotation provided as vector of Euler angles
   *
   * @return quaternion
   */
  Eigen::Quaterniond quaternionFromEuler(Eigen::Vector3d euler);
  //}

  /* Distances, Norms and Areas //{ */

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

  /**
   * @brief uses Heron's formula to compute area of a given triangle using side lengths
   *
   * @param a length of side1
   * @param b length of side2
   * @param c length of side3
   *
   * @return
   */
  double triangleArea(const double& a, const double& b, const double& c);

  /**
   * @brief computes the area of a spherical surface section in betweeen three given points
   *
   * @param a vector3
   * @param b vector3
   * @param c vector3
   *
   * @return area of the spherical surface section
   */
  double sphericalTriangleArea(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);

  Eigen::Vector2d rotateVector2d(const Eigen::Vector2d& vector_in, const double& angle);

  //}

  // | ------------------------- unused ------------------------- |

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

}  // namespace mrs_lib

#endif