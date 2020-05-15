// clang: MatousFormat
#include <mrs_lib/geometry_utils.h>

using quat_t = Eigen::Quaterniond;

namespace mrs_lib
{

  // instantiation of common template values
  vec_t<3 + 1> to_homogenous(const vec_t<3>& vec);
  vec_t<2 + 1> to_homogenous(const vec_t<2>& vec);

  /* double normalize_angle(const double angle, const double from) //{ */

  double normalize_angle(const double angle, const double from)
  {
    return normalize_angle(angle, from, 2.0 * M_PI);
  }

  //}

  /* double normalize_angle(const double angle, const double from, const double to) //{ */

  double normalize_angle(const double angle, const double from, const double to)
  {
    const double range = to - from;
    double ret = std::fmod(angle - from, range);
    if (ret < 0.0)
      ret += range;
    return ret + from;
  }

  //}

  /* double cross(const vec2_t& vec1, const vec2_t vec2) //{ */

  double cross(const vec2_t& vec1, const vec2_t vec2)
  {
    return vec1.x() * vec2.y() - vec1.y() * vec2.x();
  }

  //}

  /* double angle_between(const vec2_t& vec1, const vec2_t& vec2) and overloads //{ */

  double angle_between(const vec3_t& vec1, const vec3_t& vec2)
  {
    const double sin_12 = vec1.cross(vec2).norm();
    const double cos_12 = vec1.dot(vec2);
    const double angle = std::atan2(sin_12, cos_12);
    return angle;
  }

  double angle_between(const vec2_t& vec1, const vec2_t& vec2)
  {
    const double sin_12 = cross(vec1, vec2);
    const double cos_12 = vec1.dot(vec2);
    const double angle = std::atan2(sin_12, cos_12);
    return angle;
  }

  //}

  /* dist2d //{ */

  double dist2d(const double& ax, const double& ay, const double& bx, const double& by)
  {

    return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
  }

  //}

  /* dist3d //{ */

  double dist3d(const double& ax, const double& ay, const double& az, const double& bx, const double& by, const double& bz)
  {

    return sqrt(pow(ax - bx, 2) + pow(ay - by, 2) + +pow(az - bz, 2));
  }

  //}

  /* Eigen::AngleAxisd angleaxis_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance) //{ */

  Eigen::AngleAxisd angleaxis_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
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

  /* Eigen::Quaterniond quaternion_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance) //{ */

  Eigen::Quaterniond quaternion_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
  {
    const auto rot = angleaxis_between(vec1, vec2, tolerance);
    const Eigen::Quaterniond ret(rot);
    return ret;
  }

  //}

  /* Eigen::Matrix3d rotation_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance) //{ */

  Eigen::Matrix3d rotation_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const double tolerance)
  {
    const auto rot = angleaxis_between(vec1, vec2, tolerance);
    const Eigen::Matrix3d ret(rot);
    return ret;
  }

  //}

  /* Ray //{ */

  /* constructors //{ */
  Ray::Ray()
  {
    point1 = Eigen::Vector3d::Zero();
    point2 = Eigen::Vector3d::Zero();
  }

  Ray::Ray(Eigen::Vector3d p1, Eigen::Vector3d p2)
  {
    point1 = p1;
    point2 = p2;
  }

  Ray::~Ray()
  {
  }
  //}

  /* getters //{ */
  const Eigen::Vector3d Ray::p1()
  {
    return point1;
  }

  const Eigen::Vector3d Ray::p2()
  {
    return point2;
  }

  const Eigen::Vector3d Ray::direction()
  {
    return (point2 - point1);
  }
  //}

  /* raycasting //{ */
  Ray Ray::twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo)
  {
    return Ray(pointFrom, pointTo);
  }

  Ray Ray::directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction)
  {
    return Ray(origin, origin + direction);
  }
  //}

  //}

  /* Triangle //{ */

  /* constructors //{ */
  Triangle::Triangle()
  {
    point1 = Eigen::Vector3d(0, 0, 0);
    point2 = Eigen::Vector3d(1, 0, 0);
    point3 = Eigen::Vector3d(0, 0, 1);
  }

  Triangle::Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
  {
    point1 = a;
    point2 = b;
    point3 = c;
  }

  Triangle::~Triangle()
  {
  }
  //}

  /* getters //{ */
  const Eigen::Vector3d Triangle::a()
  {
    return point1;
  }

  const Eigen::Vector3d Triangle::b()
  {
    return point2;
  }

  const Eigen::Vector3d Triangle::c()
  {
    return point3;
  }

  const Eigen::Vector3d Triangle::normal()
  {
    Eigen::Vector3d n;
    n = (point2 - point1).cross(point3 - point1);
    return n.normalized();
  }

  const Eigen::Vector3d Triangle::center()
  {
    return (point1 + point2 + point3) / 3.0;
  }

  const std::vector<Eigen::Vector3d> Triangle::vertices()
  {
    std::vector<Eigen::Vector3d> vertices;
    vertices.push_back(point1);
    vertices.push_back(point2);
    vertices.push_back(point3);
    return vertices;
  }
  //}

  /* intersectionRay //{ */
  const boost::optional<Eigen::Vector3d> Triangle::intersectionRay(Ray r, double epsilon)
  {
    // The Möller–Trumbore algorithm
    // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    Eigen::Vector3d v1 = point2 - point1;
    Eigen::Vector3d v2 = point3 - point1;
    Eigen::Vector3d h = r.direction().cross(v2);
    double res = v1.dot(h);
    if (res > -epsilon && res < epsilon)
    {
      return boost::none;
    }
    double f = 1.0 / res;
    Eigen::Vector3d s = r.p1() - point1;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0)
    {
      return boost::none;
    }
    Eigen::Vector3d q = s.cross(v1);
    double v = f * r.direction().dot(q);
    if (v < 0.0 || u + v > 1.0)
    {
      return boost::none;
    }
    double t = f * v2.dot(q);
    if (t > epsilon)
    {
      Eigen::Vector3d ret = r.p1() + r.direction() * t;
      return ret;
    }
    return boost::none;
  }
  //}

  //}

  /* Rectangle //{ */

  /* constructors //{ */
  Rectangle::Rectangle()
  {
    point1 = Eigen::Vector3d(0, 0, 0);
    point2 = Eigen::Vector3d(1, 0, 0);
    point3 = Eigen::Vector3d(1, 1, 0);
    point4 = Eigen::Vector3d(0, 1, 0);
  }

  Rectangle::Rectangle(std::vector<Eigen::Vector3d> points)
  {
    point1 = points[0];
    point2 = points[1];
    point3 = points[2];
    point4 = points[3];
  }

  Rectangle::Rectangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d d)
  {
    point1 = a;
    point2 = b;
    point3 = c;
    point4 = d;
  }

  Rectangle::~Rectangle()
  {
  }
  //}

  /* getters //{ */
  const Eigen::Vector3d Rectangle::a()
  {
    return point1;
  }

  const Eigen::Vector3d Rectangle::b()
  {
    return point2;
  }

  const Eigen::Vector3d Rectangle::c()
  {
    return point3;
  }

  const Eigen::Vector3d Rectangle::d()
  {
    return point4;
  }

  const Eigen::Vector3d Rectangle::center()
  {
    return (point1 + point2 + point3 + point4) / 4.0;
  }

  const Eigen::Vector3d Rectangle::normal()
  {
    Eigen::Vector3d n;
    n = (point2 - point1).cross(point4 - point1);
    return n.normalized();
  }

  const std::vector<Eigen::Vector3d> Rectangle::vertices()
  {
    std::vector<Eigen::Vector3d> vertices;
    vertices.push_back(point1);
    vertices.push_back(point2);
    vertices.push_back(point3);
    vertices.push_back(point4);
    return vertices;
  }

  const std::vector<Triangle> Rectangle::triangles()
  {
    Triangle t1(point1, point2, point3);
    Triangle t2(point1, point3, point4);

    std::vector<Triangle> triangles;
    triangles.push_back(t1);
    triangles.push_back(t2);
    return triangles;
  }
  //}

  /* intersectionRay //{ */
  const boost::optional<Eigen::Vector3d> Rectangle::intersectionRay(Ray r, double epsilon)
  {
    Triangle t1 = triangles()[0];
    Triangle t2 = triangles()[1];
    auto result = t1.intersectionRay(r, epsilon);
    if (result != boost::none)
    {
      return result;
    }
    return t2.intersectionRay(r, epsilon);
  }
  //}

  /* isFacing //{ */
  bool Rectangle::isFacing(Eigen::Vector3d point)
  {
    Eigen::Vector3d towards_point = point - center();
    double dot_product = towards_point.dot(normal());
    return dot_product > 0;
  }

  //}

  //}

  /* Cuboid //{ */

  /* constructors //{ */
  Cuboid::Cuboid()
  {
    for (int i = 0; i < 8; i++)
    {
      points.push_back(Eigen::Vector3d::Zero());
    }
  }

  Cuboid::Cuboid(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, Eigen::Vector3d p4, Eigen::Vector3d p5, Eigen::Vector3d p6,
                 Eigen::Vector3d p7)
  {
    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
  }

  Cuboid::Cuboid(std::vector<Eigen::Vector3d> points)
  {
    this->points = points;
  }

  Cuboid::Cuboid(Eigen::Vector3d center, Eigen::Vector3d size, Eigen::Quaterniond orientation)
  {
    Eigen::Vector3d p0(-size.x() / 2, -size.y() / 2, -size.z() / 2);
    Eigen::Vector3d p1(size.x() / 2, -size.y() / 2, -size.z() / 2);
    Eigen::Vector3d p2(size.x() / 2, size.y() / 2, -size.z() / 2);
    Eigen::Vector3d p3(-size.x() / 2, size.y() / 2, -size.z() / 2);
    Eigen::Vector3d p4(-size.x() / 2, -size.y() / 2, size.z() / 2);
    Eigen::Vector3d p5(size.x() / 2, -size.y() / 2, size.z() / 2);
    Eigen::Vector3d p6(size.x() / 2, size.y() / 2, size.z() / 2);
    Eigen::Vector3d p7(-size.x() / 2, size.y() / 2, size.z() / 2);

    p0 = center + orientation * p0;
    p1 = center + orientation * p1;
    p2 = center + orientation * p2;
    p3 = center + orientation * p3;
    p4 = center + orientation * p4;
    p5 = center + orientation * p5;
    p6 = center + orientation * p6;
    p7 = center + orientation * p7;

    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
  }

  Cuboid::~Cuboid()
  {
  }
  //}

  /* lookupPoints //{ */
  std::vector<Eigen::Vector3d> Cuboid::lookupPoints(int face_idx)
  {
    std::vector<Eigen::Vector3d> lookup;
    switch (face_idx)
    {
      case Cuboid::BOTTOM:
        lookup.push_back(points[0]);
        lookup.push_back(points[1]);
        lookup.push_back(points[2]);
        lookup.push_back(points[3]);
        break;
      case Cuboid::TOP:
        lookup.push_back(points[4]);
        lookup.push_back(points[5]);
        lookup.push_back(points[6]);
        lookup.push_back(points[7]);
        break;
      case Cuboid::LEFT:
        lookup.push_back(points[3]);
        lookup.push_back(points[0]);
        lookup.push_back(points[4]);
        lookup.push_back(points[7]);
        break;
      case Cuboid::RIGHT:
        lookup.push_back(points[2]);
        lookup.push_back(points[6]);
        lookup.push_back(points[5]);
        lookup.push_back(points[1]);
        break;
      case Cuboid::FRONT:
        lookup.push_back(points[0]);
        lookup.push_back(points[1]);
        lookup.push_back(points[5]);
        lookup.push_back(points[4]);
        break;
      case Cuboid::BACK:
        lookup.push_back(points[2]);
        lookup.push_back(points[3]);
        lookup.push_back(points[7]);
        lookup.push_back(points[6]);
        break;
    }
    return lookup;
  }
  //}

  /* getters //{ */
  const std::vector<Eigen::Vector3d> Cuboid::vertices()
  {
    return points;
  }

  const Rectangle Cuboid::getRectangle(int face_idx)
  {
    return Rectangle(lookupPoints(face_idx));
  }

  const Eigen::Vector3d Cuboid::center()
  {
    Eigen::Vector3d point_sum = points[0];
    for (int i = 1; i < 8; i++)
    {
      point_sum += points[i];
    }
    return point_sum / 8.0;
  }
  //}

  /* intersectionRay //{ */
  const std::vector<Eigen::Vector3d> Cuboid::intersectionRay(Ray r, double epsilon)
  {
    std::vector<Eigen::Vector3d> ret;
    for (int i = 0; i < 6; i++)
    {
      Rectangle side = getRectangle(i);
      auto side_intersect = side.intersectionRay(r, epsilon);
      if (side_intersect != boost::none)
      {
        ret.push_back(side_intersect.get());
      }
    }
    return ret;
  }
  //}

  //}

  /* Ellipse //{ */

  /* constructors //{ */
  Ellipse::Ellipse()
  {
  }

  Ellipse::~Ellipse()
  {
  }

  Ellipse::Ellipse(Eigen::Vector3d center, Eigen::Quaterniond orientation, double a, double b)
  {
    center_point = center;
    absolute_orientation = orientation;
    major_semi = a;
    minor_semi = b;
  }
  //}

  /* getters //{ */
  double Ellipse::a()
  {
    return major_semi;
  }

  double Ellipse::b()
  {
    return minor_semi;
  }

  const Eigen::Vector3d Ellipse::center()
  {
    return center_point;
  }

  const Eigen::Quaterniond Ellipse::orientation()
  {
    return absolute_orientation;
  }

  //}

  //}

  /* Cylinder //{ */

  /* constructors //{ */
  Cylinder::Cylinder()
  {
  }

  Cylinder::~Cylinder()
  {
  }

  Cylinder::Cylinder(Eigen::Vector3d center, double radius, double height, Eigen::Quaterniond orientation)
  {
    this->center_point = center;
    this->radius = radius;
    this->height = height;
    this->absolute_orientation = orientation;
  }
  //}

  /* getters //{ */
  const Eigen::Vector3d Cylinder::center()
  {
    return center_point;
  }

  const Eigen::Quaterniond Cylinder::orientation()
  {
    return absolute_orientation;
  }

  double Cylinder::r()
  {
    return radius;
  }

  double Cylinder::h()
  {
    return height;
  }

  const Ellipse Cylinder::getCap(int index)
  {
    Ellipse e;
    Eigen::Vector3d ellipse_center;
    switch (index)
    {
      case Cylinder::BOTTOM:
        ellipse_center = center() - orientation() * (Eigen::Vector3d::UnitZ() * (h() / 2.0));
        e = Ellipse(ellipse_center, orientation(), r(), r());
        break;
      case Cylinder::TOP:
        ellipse_center = center() + orientation() * (Eigen::Vector3d::UnitZ() * (h() / 2.0));
        e = Ellipse(ellipse_center, orientation(), r(), r());
        break;
    }
    return e;
  }

  //}

  //}

  /* Cone //{ */

  /* constructors //{ */
  Cone::Cone()
  {
  }

  Cone::~Cone()
  {
  }

  Cone::Cone(Eigen::Vector3d origin_point, double angle, double height, Eigen::Vector3d absolute_direction)
  {
    this->origin_point = origin_point;
    this->angle = angle;
    this->height = height;
    this->absolute_direction = absolute_direction.normalized();
  }
  //}

  /* getters //{ */
  const Eigen::Vector3d Cone::origin()
  {
    return origin_point;
  }

  const Eigen::Vector3d Cone::direction()
  {
    return absolute_direction;
  }

  const Eigen::Vector3d Cone::center()
  {
    return origin() + (0.5 * h()) * direction();
  }

  double Cone::theta()
  {
    return angle;
  }

  double Cone::h()
  {
    return height;
  }

  const Ellipse Cone::getCap()
  {
    Eigen::Vector3d ellipse_center = origin() + direction() * h();
    Eigen::Quaterniond ellipse_orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction());
    double cap_radius = std::tan(theta()) * h();
    Ellipse e(ellipse_center, ellipse_orientation, cap_radius, cap_radius);
    return e;
  }

  //}

  //}

  // | --------------------- angle routines --------------------- |

  /* haversin() //{ */

  double haversin(const double& angle)
  {
    return (1.0 - std::cos(angle)) / 2.0;
  }

  //}

  /* invHaversin() //{ */

  double invHaversin(const double& value)
  {
    return 2.0 * std::asin(std::sqrt(value));
  }

  //}

  /* triangleArea() //{ */

  double triangleArea(const double& a, const double& b, const double& c)
  {
    double s = (a + b + c) / 2.0;
    return std::sqrt(s * (s - a) * (s - b) * (s - c));
  }

  //}

  /* vectorAngle() //{ */

  double vectorAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
  {
    double scalar = v1.dot(v2) / (v1.norm() * v2.norm());

    Eigen::Vector3d axis = v1.cross(v2);

    double dir = v2.dot(axis.cross(v1));
    double sgn = sign(dir);

    // singularities
    if (scalar > 1.0)
      return 0;
    else if (scalar < -1.0)
      return sgn * M_PI;

    return sgn * acos(scalar);
  }

  //}

  /* solidAngle() //{ */

  double solidAngle(const double& a, const double& b, const double& c)
  {
    return invHaversin((haversin(c) - haversin(a - b)) / (std::sin(a) * std::sin(b)));
  }

  //}

  /* sphericalTriangleArea() //{ */

  double sphericalTriangleArea(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
  {
    double ab = vectorAngle(a, b);
    double bc = vectorAngle(b, c);
    double ca = vectorAngle(c, a);

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

  /* rectSolidAngle() //{ */

  double rectSolidAngle(Rectangle r, Eigen::Vector3d center)
  {
    Eigen::Vector3d a = r.a() - center;
    Eigen::Vector3d b = r.b() - center;
    Eigen::Vector3d c = r.c() - center;
    Eigen::Vector3d d = r.d() - center;

    a.normalize();
    b.normalize();
    c.normalize();
    d.normalize();

    double t1 = sphericalTriangleArea(a, b, c);
    double t2 = sphericalTriangleArea(c, d, a);

    return t1 + t2;
  }
  //}

  /* unwrapAngle() //{ */

  double unwrapAngle(const double& yaw, const double& yaw_previous)
  {

    double yaw_out = yaw;

    for (int i = 0; (i < 10) && (yaw_out - yaw_previous > M_PI); i++)
    {
      yaw_out -= 2 * M_PI;
    }

    for (int i = 0; (i < 10) && (yaw_out - yaw_previous < -M_PI); i++)
    {
      yaw_out += 2 * M_PI;
    }

    return yaw_out;
  }

  //}

  /* wrapAngle() //{ */

  double wrapAngle(const double& angle_in)
  {

    double angle_wrapped = angle_in;

    for (int i = 0; (i < 10) && (angle_wrapped > M_PI); i++)
    {
      angle_wrapped -= 2 * M_PI;
    }

    for (int i = 0; (i < 10) && (angle_wrapped < -M_PI); i++)
    {
      angle_wrapped += 2 * M_PI;
    }

    return angle_wrapped;
  }

  //}

  /* disambiguateAngle() //{ */

  double disambiguateAngle(const double& yaw, const double& yaw_previous)
  {

    if (yaw - yaw_previous > M_PI / 2)
    {
      return yaw - M_PI;
    } else if (yaw - yaw_previous < -M_PI / 2)
    {
      return yaw + M_PI;
    }

    return yaw;
  }

  //}

  /* angleBetween() //{ */

  double angleBetween(const double& a, const double& b)
  {

    double temp = a - b;

    return atan2(sin(temp), cos(temp));
  }

  //}

  /* interpolateAngles() //{ */

  double interpolateAngles(const double& a1, const double& a2, const double& coeff)
  {

    // interpolate the yaw
    Eigen::Vector3d axis = Eigen::Vector3d(0, 0, 1);

    Eigen::Quaterniond quat1 = Eigen::Quaterniond(Eigen::AngleAxis<double>(a1, axis));
    Eigen::Quaterniond quat2 = Eigen::Quaterniond(Eigen::AngleAxis<double>(a2, axis));

    quat_t new_quat = quat1.slerp(coeff, quat2);

    Eigen::Vector3d vecx = new_quat * Eigen::Vector3d(1, 0, 0);

    return atan2(vecx[1], vecx[0]);
  }

  //}

  /* rotateVector2d() //{ */

  Eigen::Vector2d rotateVector2d(const Eigen::Vector2d& vector_in, const double& angle)
  {

    Eigen::Rotation2D<double> rot2(angle);

    return rot2.toRotationMatrix() * vector_in;
  }

  //}

  /* quaternionFromEuler //{ */
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

  // | ------------ not finished / not used / broken ------------ |

  /* UNUSED //{ */

  /* /1* ConicSection constructor //{ *1/ */
  /* ConicSection::ConicSection(const double A, const double B, const double C, const double D, const double E, const double F, const double tolerance) */
  /* { */
  /*   // determinant of the 2x2 submatrix (the discriminant) */
  /*   alpha = B*B - 4*A*C; */

  /*   // determinant of the 3x3 matrix */
  /*   beta = (B*E*D - C*D*D - A*E*E - F*alpha)/4.0; */

  /*   conic_type = classify_conic_section(alpha, beta, tolerance); */
  /* } */
  /* //} */

  /* /1* classify_conic_section() method //{ *1/ */
  /* ConicSection::conic_type_t ConicSection::classify_conic_section(const double alpha, const double beta, const double tolerance) */
  /* { */
  /*   conic_type_t conic_type; */
  /*   if (abs(beta) < tolerance) */
  /*   { */
  /*     if (abs(alpha) < tolerance) */
  /*       conic_type = parallel_lines; */
  /*     else if (alpha < 0.0) */
  /*       conic_type = point; */
  /*     else // (alpha > 0.0) */
  /*       conic_type = intersecting_lines; */
  /*   } else */
  /*   { */
  /*     if (abs(alpha) < tolerance) */
  /*       conic_type = parabola; */
  /*     else if (alpha < 0.0) */
  /*     { */
  /*       if (abs(A-C) < tolerance && abs(B) < tolerance) */
  /*         conic_type = circle; */
  /*       else */
  /*         conic_type = ellipse; */
  /*     } */
  /*     else // (alpha > 0.0) */
  /*     { */
  /*       if (abs(A+C) < tolerance) */
  /*         conic_type = rectangular_hyperbola; */
  /*       else */
  /*         conic_type = hyperbola; */
  /*     } */
  /*   } */
  /*   return conic_type; */
  /* } */
  /* //} */

  // THIS IS WRONG!! It's actually pretty complicated stuff!!
  /* double ConicSection::closest_point_on_ellipse_param(const pt2_t& to_point, const central_conic_params_t& ellipse_params) */
  /* { */
  /*   // transform the point to a frame where the ellipse is centered and has axes aligned with the frame axes */
  /*   const Eigen::Rotation2Dd rot(-ellipse_params.theta); */
  /*   const pt2_t pt_tfd = rot*(to_point - vec2_t(ellipse_params.x0, ellipse_params.y0)); */
  /*   // calculate the `t` parameter of the ellipse, corresponding to the angle in which the point lies */
  /*   const double t = atan2(ellipse_params.a*pt_tfd.y(), ellipse_params.b*pt_tfd.x()); */
  /*   return t; */
  /* } */

  /* /1* calc_central_conic_params() method //{ *1/ */
  /* ConicSection::central_conic_params_t ConicSection::calc_central_conic_params() */
  /* { */
  /*   assert(conic_type & basic_type_mask == ellipse || conic_type & basic_type_mask == hyperbola); */
  /*   const double tmp1 = 2*(A*E*E + C*D*D - B*D*E + alpha*F); */
  /*   const double tmp2 = sqrt((A - C)*(A - C) + B*B); */
  /*   const double atmp = tmp1*((A + C) + tmp2); */
  /*   const double btmp = tmp1*((A + C) - tmp2); */
  /*   const double a = sqrt(abs(atmp))/alpha; */
  /*   const double b = sqrt(abs(btmp))/alpha; */
  /*   const double x0 = (2*C*D - B*E)/alpha; */
  /*   const double y0 = (2*A*E - B*D)/alpha; */
  /*   double th = atan2(C - A - tmp2, B); */
  /*   if (A < C) */
  /*     th += M_PI_2; */
  /*   central_conic_params_t ret; */
  /*   ret.a = a; */
  /*   ret.b = b; */
  /*   ret.x0 = x0; */
  /*   ret.y0 = y0; */
  /*   ret.theta = th; */
  /*   return ret; */
  /* } */
  /* //} */

  //}

  /* NOT FINISHED //{ */

  /* // TODO: probably doesn't do what it claims */
  /* std::optional<Eigen::Vector3d> Cone::ProjectPointOnPlane(Plane plane, Eigen::Vector3d point) { */

  /*   std::optional<Eigen::Vector3d> intersect = plane.intersectionRay(this->cone_ray, 1e-6); */

  /*   if (!intersect) { */
  /*     return intersect; */
  /*   } */

  /*   Eigen::Vector3d vec_to_intersect = intersect.value() - this->position; */
  /*   vec_to_intersect.normalize(); */

  /*   Eigen::Vector3d vec_to_point = intersect.value() - this->position; */
  /*   vec_to_point.normalize(); */

  /*   Eigen::Vector3d my_axis = this->direction.cross(vec_to_point); */
  /*   my_axis.normalize(); */

  /*   Eigen::AngleAxis<double> my_quat(this->angle, my_axis); */

  /*   Eigen::Vector3d vec_along_cone = my_quat * vec_to_intersect; */
  /*   Ray ray_along_cone(this->position, this->position + vec_along_cone); */

  /*   std::optional<Eigen::Vector3d> intersect2 = plane.intersectionRay(ray_along_cone, 1e-6); */

  /*   return intersect2; */
  /* } */

  //}

}  // namespace mrs_lib
