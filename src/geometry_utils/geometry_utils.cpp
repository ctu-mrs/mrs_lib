// clang: MatousFormat
#include <mrs_lib/geometry_utils.h>

namespace mrs_lib
{

  /* double angle_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) //{ */

  double angle_between(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2)
  {
    const Eigen::Vector3d a = vec1.normalized();
    const Eigen::Vector3d b = vec2.normalized();
    const Eigen::Vector3d v = a.cross(b);
    const double sin_ab = v.norm();
    const double cos_ab = a.dot(b);
    const double angle = std::atan2(sin_ab, cos_ab);
    return angle;
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

  Ray::Ray()
  {
    this->origin = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->direction = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->energy = 0.0;
  }

  Ray::~Ray()
  {
  }

  Ray::Ray(Eigen::Vector3d origin, Eigen::Vector3d direction, double energy)
  {
    this->origin = origin;
    this->direction = direction;
    this->energy = energy;
  }

  //}

  /* Plane //{ */

  Plane::Plane()
  {
  }

  Plane::Plane(Eigen::Vector3d point, Eigen::Vector3d normal)
  {
    this->point = point;
    this->normal = normal;
  }

  Plane::~Plane()
  {
  }

  std::optional<Eigen::Vector3d> Plane::intersectionRay(Ray r, double epsilon)
  {

    double denom = this->normal.dot(r.direction);

    if (fabs(denom) < epsilon)
    {

      return std::optional<Eigen::Vector3d>{};
    }

    Eigen::Vector3d p0l0 = this->point - r.origin;
    double t = this->normal.dot(p0l0) / denom;

    if (t >= 0)
    {
      return Eigen::Vector3d(t * r.direction + r.origin);
    } else
    {
      return std::optional<Eigen::Vector3d>{};
    }
  }

  //}

  /* Rectangle //{ */

  Rectangle::Rectangle()
  {
  }

  Rectangle::~Rectangle()
  {
  }

  Rectangle::Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D)
  {

    Eigen::Vector3d v1 = D - A;
    Eigen::Vector3d v2 = D - C;

    this->points.push_back(A);
    this->points.push_back(B);
    this->points.push_back(C);
    this->points.push_back(D);

    this->normal_vector = v1.cross(v2);
    this->normal_vector.normalize();

    if (A == B || A == C || A == D || B == C || B == D || C == D)
    {
      return;
    }

    this->plane = Plane(A, normal_vector);

    this->basis.col(0) << D - A;
    this->basis.col(1) << D - C;
    this->basis.col(2) << normal_vector;

    this->projector = basis * basis.transpose();
  }

  std::optional<Eigen::Vector3d> Rectangle::intersectionRay(Ray r, double epsilon)
  {

    std::optional<Eigen::Vector3d> intersect = this->plane.intersectionRay(r, epsilon);
    if (!intersect)
    {
      return intersect;
    }

    Eigen::Vector3d projection = basis.inverse() * (points[3] - intersect.value());
    if (projection[0] >= 0.0 && projection[0] <= 1.0 && projection[1] >= 0.0 && projection[1] <= 1.0)
    {
      return intersect;
    }

    return std::optional<Eigen::Vector3d>{};
  }

  //}

  /* Angle routines //{ */

  double haversin(double angle)
  {
    return (1.0 - std::cos(angle)) / 2.0;
  }

  double invHaversin(double angle)
  {
    return 2.0 * std::asin(std::sqrt(angle));
  }

  double triangleArea(double a, double b, double c)
  {
    double s = (a + b + c) / 2.0;
    return std::sqrt(s * (s - a) * (s - b) * (s - c));
  }

  double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2)
  {
    return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
  }

  double solidAngle(double a, double b, double c)
  {
    return invHaversin((haversin(c) - haversin(a - b)) / (std::sin(a) * std::sin(b)));
  }

  double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
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

  double rectSolidAngle(Rectangle r, Eigen::Vector3d center)
  {
    Eigen::Vector3d a = r.points[0] - center;
    Eigen::Vector3d b = r.points[1] - center;
    Eigen::Vector3d c = r.points[2] - center;
    Eigen::Vector3d d = r.points[3] - center;

    a.normalize();
    b.normalize();
    c.normalize();
    d.normalize();

    double t1 = sphericalTriangleArea(a, b, c);
    double t2 = sphericalTriangleArea(c, d, a);

    return t1 + t2;
  }

  //}

  /* Cone //{ */

  Cone::Cone(Eigen::Vector3d position, Eigen::Vector3d direction, double angle)
  {

    this->position = position;
    this->direction = direction;
    this->angle = angle;

    this->cone_axis_projector = this->direction * this->direction.transpose();

    this->cone_ray = Ray(position, direction, 0);
  }

  Cone::~Cone()
  {
  }

  Eigen::Vector3d Cone::ProjectPoint(Eigen::Vector3d point)
  {

    Eigen::Vector3d point_vec = point - this->position;
    double point_axis_angle = acos((point_vec.dot(this->direction)) / (point_vec.norm() * this->direction.norm()));

    /* Eigen::Vector3d axis_projection = this->cone_axis_projector * point_vec + this->position; */

    Eigen::Vector3d axis_rot = this->direction.cross(point_vec);
    axis_rot.normalize();

    Eigen::AngleAxis<double> my_quat(this->angle - point_axis_angle, axis_rot);

    Eigen::Vector3d point_on_cone = my_quat * point_vec + this->position;

    Eigen::Vector3d vec_point_on_cone = point_on_cone - this->position;
    vec_point_on_cone.normalize();

    double beta = this->angle - point_axis_angle;

    if (point_axis_angle < this->angle)
    {
      return this->position + vec_point_on_cone * cos(beta) * point_vec.norm();
    } else if ((point_axis_angle >= this->angle) && (point_axis_angle - this->angle) <= M_PI / 2.0)
    {  // TODO: is this condition correct?
      return this->position + vec_point_on_cone * cos(point_axis_angle - this->angle) * point_vec.norm();
    } else
    {
      return this->position;
    }
  }

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
