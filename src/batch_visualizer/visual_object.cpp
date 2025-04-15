#include <mrs_lib/visual_object.h>

namespace mrs_lib
{

/* utils //{ */

/* conversions //{ */
geometry_msgs::msg::Point eigenToMsg(const Eigen::Vector3d& v) {
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

std_msgs::msg::ColorRGBA generateColor(const double r, const double g, const double b, const double a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Eigen::Vector3d msgToEigen(const geometry_msgs::msg::Point& p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}
//}

/* buildEllipse //{ */
std::vector<Eigen::Vector3d> buildEllipse(const mrs_lib::geometry::Ellipse& ellipse, const int num_points) {
  std::vector<Eigen::Vector3d> points;
  double                       theta = 0;
  for (int i = 0; i < num_points; i++) {
    double          nom = (ellipse.a() * ellipse.b());
    double          den = sqrt(((ellipse.b() * cos(theta)) * (ellipse.b() * cos(theta))) + ((ellipse.a() * sin(theta)) * (ellipse.a() * sin(theta))));
    double          rho = nom / den;
    Eigen::Vector3d point(rho * cos(theta), rho * sin(theta), 0);
    point = ellipse.center() + ellipse.orientation() * point;
    points.push_back(point);
    theta += 2.0 * M_PI / num_points;
  }
  return points;
}
//}

//}

/* addRay //{ */
void VisualObject::addRay(const mrs_lib::geometry::Ray& ray, const double r, const double g, const double b, const double a) {
  type_ = MarkerType::LINE;
  points_.push_back(eigenToMsg(ray.p1()));
  points_.push_back(eigenToMsg(ray.p2()));
  colors_.push_back(generateColor(r, g, b, a));
  colors_.push_back(generateColor(r, g, b, a));
}
//}

/* addTriangle //{ */
void VisualObject::addTriangle(const mrs_lib::geometry::Triangle& triangle, const double r, const double g, const double b, const double a, const bool filled) {
  if (filled) {
    type_ = MarkerType::TRIANGLE;
    points_.push_back(eigenToMsg(triangle.a()));
    points_.push_back(eigenToMsg(triangle.b()));
    points_.push_back(eigenToMsg(triangle.c()));
    colors_.push_back(generateColor(r, g, b, a));
    colors_.push_back(generateColor(r, g, b, a));
    colors_.push_back(generateColor(r, g, b, a));
  } else {
    type_ = MarkerType::LINE;
    points_.push_back(eigenToMsg(triangle.a()));
    points_.push_back(eigenToMsg(triangle.b()));
    colors_.push_back(generateColor(r, g, b, a));
    colors_.push_back(generateColor(r, g, b, a));

    points_.push_back(eigenToMsg(triangle.b()));
    points_.push_back(eigenToMsg(triangle.c()));
    colors_.push_back(generateColor(r, g, b, a));
    colors_.push_back(generateColor(r, g, b, a));

    points_.push_back(eigenToMsg(triangle.c()));
    points_.push_back(eigenToMsg(triangle.a()));
    colors_.push_back(generateColor(r, g, b, a));
    colors_.push_back(generateColor(r, g, b, a));
  }
}
//}

/* addEllipse //{ */
void VisualObject::addEllipse(const mrs_lib::geometry::Ellipse& ellipse, const double r, const double g, const double b, const double a, const bool filled, const int num_points) {

  std::vector<Eigen::Vector3d> points = buildEllipse(ellipse, num_points);
  if (filled) {
    for (int i = 0; i < num_points - 1; i++) {
      mrs_lib::geometry::Triangle tri(ellipse.center(), points[i], points[i + 1]);
      addTriangle(tri, r, g, b, a, true);
    }
    mrs_lib::geometry::Triangle tri(ellipse.center(), points[num_points - 1], points[0]);
    addTriangle(tri, r, g, b, a, true);

  } else {
    for (int i = 0; i < num_points - 1; i++) {
      mrs_lib::geometry::Ray ray = mrs_lib::geometry::Ray::twopointCast(points[i], points[i + 1]);
      addRay(ray, r, g, b, a);
    }
    mrs_lib::geometry::Ray ray = mrs_lib::geometry::Ray::twopointCast(points[num_points - 1], points[0]);
    addRay(ray, r, g, b, a);
  }
}
//}

/* Eigen::Vector3d //{ */
VisualObject::VisualObject(const Eigen::Vector3d& point, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const unsigned long& id, const rclcpp::Node::SharedPtr& node) : id_(id), node_(node) {

  type_ = MarkerType::POINT;
  points_.push_back(eigenToMsg(point));
  colors_.push_back(generateColor(r, g, b, a));

  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}

//}

/* mrs_lib::geometry::Ray //{ */
VisualObject::VisualObject(const mrs_lib::geometry::Ray& ray, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const unsigned long& id, const rclcpp::Node::SharedPtr& node) : id_(id), node_(node) {
  type_ = MarkerType::LINE;
  addRay(ray, r, g, b, a);
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_lib::geometry::Triangle //{ */
VisualObject::VisualObject(const mrs_lib::geometry::Triangle& triangle, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node) : id_(id), node_(node) {
  addTriangle(triangle, r, g, b, a, filled);
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_lib::geometry::Rectangle //{ */
VisualObject::VisualObject(const mrs_lib::geometry::Rectangle& rectangle, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node) : id_(id), node_(node) {
  for (const auto& t : rectangle.triangles()) {
    addTriangle(t, r, g, b, a, filled);
  }
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_lib::geometry::Cuboid //{ */
VisualObject::VisualObject(const mrs_lib::geometry::Cuboid& cuboid, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node) : id_(id), node_(node) {

  for (int i = 0; i < 6; i++) {
    for (const auto& t : cuboid.getRectangle(i).triangles()) {
      addTriangle(t, r, g, b, a, filled);
    }
  }
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_lib::geometry::Ellipse//{ */
VisualObject::VisualObject(const mrs_lib::geometry::Ellipse& ellipse, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node, const int num_points) : id_(id), node_(node) {
  addEllipse(ellipse, r, g, b, a, filled, num_points);
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_lib::geometry::Cylinder //{ */
VisualObject::VisualObject(const mrs_lib::geometry::Cylinder& cylinder, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const bool capped, const unsigned long& id, const rclcpp::Node::SharedPtr& node, const int num_sides) : id_(id), node_(node) {
  if (capped) {
    mrs_lib::geometry::Ellipse top    = cylinder.getCap(mrs_lib::geometry::Cylinder::TOP);
    mrs_lib::geometry::Ellipse bottom = cylinder.getCap(mrs_lib::geometry::Cylinder::BOTTOM);
    addEllipse(top, r, g, b, a, filled, num_sides);
    addEllipse(bottom, r, g, b, a, filled, num_sides);
  }
  std::vector<Eigen::Vector3d> top_points    = buildEllipse(cylinder.getCap(mrs_lib::geometry::Cylinder::TOP), num_sides);
  std::vector<Eigen::Vector3d> bottom_points = buildEllipse(cylinder.getCap(mrs_lib::geometry::Cylinder::BOTTOM), num_sides);
  for (unsigned int i = 0; i < top_points.size() - 1; i++) {
    mrs_lib::geometry::Rectangle rect(bottom_points[i], bottom_points[i + 1], top_points[i + 1], top_points[i]);
    addTriangle(rect.triangles()[0], r, g, b, a, filled);
    addTriangle(rect.triangles()[1], r, g, b, a, filled);
  }
  mrs_lib::geometry::Rectangle rect(bottom_points[bottom_points.size() - 1], bottom_points[0], top_points[0], top_points[top_points.size() - 1]);
  addTriangle(rect.triangles()[0], r, g, b, a, filled);
  addTriangle(rect.triangles()[1], r, g, b, a, filled);
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_lib::geometry::Cone //{ */
VisualObject::VisualObject(const mrs_lib::geometry::Cone& cone, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const bool capped, const unsigned long& id, const rclcpp::Node::SharedPtr& node, const int num_sides) : id_(id), node_(node) {
  if (capped) {
    mrs_lib::geometry::Ellipse cap = cone.getCap();
    addEllipse(cap, r, g, b, a, filled, num_sides);
  }
  std::vector<Eigen::Vector3d> cap_points = buildEllipse(cone.getCap(), num_sides);
  for (unsigned int i = 0; i < cap_points.size() - 1; i++) {
    mrs_lib::geometry::Triangle tri(cap_points[i], cap_points[i + 1], cone.origin());
    addTriangle(tri, r, g, b, a, filled);
  }
  mrs_lib::geometry::Triangle tri(cap_points[cap_points.size() - 1], cap_points[0], cone.origin());
  addTriangle(tri, r, g, b, a, filled);
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_msgs::msg::Path //{ */
VisualObject::VisualObject(const mrs_msgs::msg::Path& p, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node) : id_(id), node_(node) {
  if (p.points.size() < 2) {
    return;
  }
  if (filled) {
    for (size_t i = 0; i < p.points.size() - 1; i++) {
      Eigen::Vector3d p1, p2;
      p1.x()   = p.points[i].position.x;
      p1.y()   = p.points[i].position.y;
      p1.z()   = p.points[i].position.z;
      p2.x()   = p.points[i + 1].position.x;
      p2.y()   = p.points[i + 1].position.y;
      p2.z()   = p.points[i + 1].position.z;
      auto ray = mrs_lib::geometry::Ray::twopointCast(p1, p2);
      addRay(ray, r, g, b, a);
    }
  } else {
    type_ = MarkerType::POINT;
    for (size_t i = 0; i < p.points.size(); i++) {
      points_.push_back(p.points[i].position);
      colors_.push_back(generateColor(r, g, b, a));
    }
  }
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* mrs_msgs::msg::TrajectoryReference //{ */
VisualObject::VisualObject(const mrs_msgs::msg::TrajectoryReference& traj, const double r, const double g, const double b, const double a, const rclcpp::Duration& timeout, const bool filled, const unsigned long& id, const rclcpp::Node::SharedPtr& node) : id_(id), node_(node) {
  if (traj.points.size() < 2) {
    return;
  }
  if (filled) {
    for (size_t i = 0; i < traj.points.size() - 1; i++) {
      Eigen::Vector3d p1, p2;
      p1.x()   = traj.points[i].position.x;
      p1.y()   = traj.points[i].position.y;
      p1.z()   = traj.points[i].position.z;
      p2.x()   = traj.points[i + 1].position.x;
      p2.y()   = traj.points[i + 1].position.y;
      p2.z()   = traj.points[i + 1].position.z;
      auto ray = mrs_lib::geometry::Ray::twopointCast(p1, p2);
      addRay(ray, r, g, b, a);
    }
  } else {
    type_ = MarkerType::POINT;
    for (size_t i = 0; i < traj.points.size(); i++) {
      points_.push_back(traj.points[i].position);
      colors_.push_back(generateColor(r, g, b, a));
    }
  }
  if (timeout.seconds() <= 0) {
    timeout_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  } else {
    timeout_time_ = node_->now() + timeout;
  }
}
//}

/* getID //{ */
unsigned long VisualObject::getID() const {
  return id_;
}
//}

/* getType //{ */
int VisualObject::getType() const {
  return type_;
}
//}

/* isTimedOut //{ */
bool VisualObject::isTimedOut() const {
  return timeout_time_.seconds() <= 0 && (timeout_time_ - node_->now()).seconds() <= 0;
}
//}

/* getPoints //{ */
const std::vector<geometry_msgs::msg::Point> VisualObject::getPoints() const {
  return points_;
}
//}

/* getColors //{ */
const std::vector<std_msgs::msg::ColorRGBA> VisualObject::getColors() const {
  return colors_;
}
//}

}  // namespace mrs_lib
