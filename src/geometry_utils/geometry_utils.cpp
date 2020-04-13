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

  // | --------------------- angle routines --------------------- |

  /* haversin() //{ */

  double haversin(const double& angle)
  {
    return (1.0 - std::cos(angle)) / 2.0;
  }

  //}

  /* invHaversin() //{ */

  double invHaversin(const double& angle)
  {
    return 2.0 * std::asin(std::sqrt(angle));
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

  double rectSolidAngle(const Rectangle& r, const Eigen::Vector3d& center)
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
