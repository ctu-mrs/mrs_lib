// clang: MatousFormat
/**  \file
     \brief Defines various geometrical shapes and their relations.
     \author Petr Štibinger - stibipet@fel.cvut.cz
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef SHAPES_H
#define SHAPES_H

#include <optional>

#include <Eigen/Dense>

namespace mrs_lib
{
  namespace geometry
  {

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
      const Eigen::Vector3d p1() const;
      /**
       * @brief get the end point
       *
       * @return ray end point
       */
      const Eigen::Vector3d p2() const;

      /**
       * @brief get the direction of ray (normalized)
       *
       * @return direction (normalized)
       */
      const Eigen::Vector3d direction() const;

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
      /**
       * @brief getter for first point
       *
       * @return 1st point (vector3)
       */
      const Eigen::Vector3d a() const;

      /**
       * @brief getter for second point
       *
       * @return 2nd point (vector3)
       */
      const Eigen::Vector3d b() const;

      /**
       * @brief getter for third point
       *
       * @return 3rd point (vector3)
       */
      const Eigen::Vector3d c() const;

      /**
       * @brief get position on the triangle center
       *
       * @return vector3
       */
      const Eigen::Vector3d center() const;

      /**
       * @brief get normal vector of this triangle. The vector origin is placed at triangle center, length is normalized and direction follows the right-hand
       * rule with respect to vertex order a-b-c
       *
       * @return vector3
       */
      const Eigen::Vector3d normal() const;

      /**
       * @brief get a vector of all vertices
       *
       * @return std::vector<vector3>
       */
      const std::vector<Eigen::Vector3d> vertices() const;

    public:
      /**
       * @brief calculate an intersection of this triangle with a given ray with given tolerance
       *
       * @param r ray to calculate intersection with
       * @param epsilon calculation tolerance
       *
       * @return vector3 intersection if exists, std::none if no intersection is found
       */
      const std::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4) const;
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
      const Eigen::Vector3d a() const;

      /**
       * @brief getter for the second point
       *
       * @return 2nd point (vector3)
       */
      const Eigen::Vector3d b() const;

      /**
       * @brief getter for the third point
       *
       * @return 3rd point (vector3)
       */
      const Eigen::Vector3d c() const;

      /**
       * @brief getter for the fourth point
       *
       * @return 4th point (vector3)
       */
      const Eigen::Vector3d d() const;

      /**
       * @brief getter for center point
       *
       * @return center point (vector3)
       */
      const Eigen::Vector3d center() const;

      /**
       * @brief getter for the normal vector. It originates in the center of the Rectangle, length is normalized, orientation follows the right-hand rule,
       * assuiming the points are provided in counter-clockwise order
       *
       * @return normal vector3
       */
      const Eigen::Vector3d normal() const;

      /**
       * @brief getter for all the points of this rectangle provided as std::vector
       *
       * @return std::vector of points (vector3)
       */
      const std::vector<Eigen::Vector3d> vertices() const;

      /**
       * @brief getter for the triangles forming this rectangle
       *
       * @return std::vector of triangles
       */
      const std::vector<Triangle> triangles() const;

      /**
       * @brief calculate an intersection of this rectangle with a given ray with given tolerance
       *
       * @param r ray to calculate intersection with
       * @param epsilon calculation tolerance
       *
       * @return vector3 intersection if exists, std::none if no intersection is found
       */
      const std::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4) const;

      /**
       * @brief check if the normal is facing a given point, i.e. if the point lies in the same half-space as the rectangle normal
       *
       * @param point vector3 to check against
       *
       * @return true if the normal is facing given point. Returns false for angle >= 90 degrees
       */
      bool isFacing(Eigen::Vector3d point) const;


      /**
       * @brief compute the solid angle of this rectangle relative to a given sphere center
       *
       * @param point center of a sphere to compute the solid angle for
       *
       * @return solid angle in steradians
       */
      double solidAngleRelativeTo(Eigen::Vector3d point) const;
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

      /**
       * @brief return points in a side of given index
       *
       * @param face_idx index of the cuboid side
       *
       * @return std::vector<vector3>
       */
      std::vector<Eigen::Vector3d> lookupPoints(int face_idx) const;

    public:
      /**
       * @brief getter for all vertices (vector3) of this cuboid
       *
       * @return std::vector<vector3>
       */
      const std::vector<Eigen::Vector3d> vertices() const;

      /**
       * @brief getter for the center point
       *
       * @return vector3
       */
      const Eigen::Vector3d center() const;

      /**
       * @brief getter for one side corresponding to a provided index
       *
       * @param face_idx index of the side to lookup points for
       *
       * @return rectangle representing the cuboid side
       */
      const Rectangle getRectangle(int face_idx) const;

      /**
       * @brief calculate the intersection between this cuboid and a provided ray within a given tolerance. Can result in 0, 1 or 2 intersection points
       *
       * @param r ray to check intersection with
       * @param epsilon tolerance for the calculation
       *
       * @return std::vector<vector3> list of intersection points (depending on the geometry, can yield 0, 1 or 2 points)
       */
      const std::vector<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-4) const;
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
      double a() const;

      /**
       * @brief getter for minor semi-axis
       *
       * @return length of minor semi-axis
       */
      double b() const;

      /**
       * @brief getter for the center point
       *
       * @return vector3
       */
      const Eigen::Vector3d center() const;

      /**
       * @brief getter for the orientation
       *
       * @return quaternion
       */
      const Eigen::Quaterniond orientation() const;
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
      const Eigen::Vector3d center() const;

      /**
       * @brief getter for the orientation
       *
       * @return quaternion
       */
      const Eigen::Quaterniond orientation() const;

      /**
       * @brief getter for cap radius
       *
       * @return radius of the cap
       */
      double r() const;

      /**
       * @brief getter for the body height
       *
       * @return body height
       */
      double h() const;

      /**
       * @brief getter for a cap corresponding to a provided index
       *
       * @param index of the cap
       *
       * @return ellipse representing the cap
       */
      const Ellipse getCap(int index) const;
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
      const Eigen::Vector3d origin() const;

      /**
       * @brief getter for the direction. Normalized direction from origin towards base
       *
       * @return vector3, normalized
       */
      const Eigen::Vector3d direction() const;

      /**
       * @brief getter for the center point. Center point lies in half of the body height
       *
       * @return vector3
       */
      const Eigen::Vector3d center() const;

      /**
       * @brief getter for angle between body height and body side
       *
       * @return angle in radians
       */
      double theta() const;

      /**
       * @brief getter for body height
       *
       * @return
       */
      double h() const;

      /**
       * @brief getter for the cap of the cone
       *
       * @return ellipse representing the cap of the cone
       */
      const Ellipse getCap() const;

      /**
       * @brief Project a 3D point orthogonally onto the Cone surface
       *
       * @param point in 3D
       *
       * @return projected point
       */
      const std::optional<Eigen::Vector3d> projectPoint(const Eigen::Vector3d& point) const;
    };
    //}
  }  // namespace geometry
}  // namespace mrs_lib

#endif  // SHAPES_H
