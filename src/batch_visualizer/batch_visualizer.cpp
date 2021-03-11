#include <string>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/geometry/shapes.h>

namespace mrs_lib
{

/* conversion utils //{ */
geometry_msgs::Point eigenToMsg(const Eigen::Vector3d v) {
  geometry_msgs::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

std_msgs::ColorRGBA generateColor(const double r, const double g, const double b, const double a) {
  std_msgs::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Eigen::Vector3d msgToEigen(const geometry_msgs::Point p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}
//}

bool   reconfigured     = false;
double tmp_points_scale = 0.02;
double tmp_lines_scale  = 0.04;

/* constructors */  //{
BatchVisualizer::BatchVisualizer() {
}

BatchVisualizer::~BatchVisualizer() {
}

BatchVisualizer::BatchVisualizer(ros::NodeHandle& nh, std::string marker_topic_name, std::string parent_frame) {
  this->parent_frame      = parent_frame;
  this->marker_topic_name = marker_topic_name;
  initialize(nh);

  this->visual_pub = nh.advertise<visualization_msgs::MarkerArray>(marker_topic_name.c_str(), 1);
  publish();
  ros::spinOnce();
}
//}

/* setParentFrame() //{ */

void BatchVisualizer::setParentFrame(const std::string parent_frame) {
  this->parent_frame               = parent_frame;
  points_marker.header.frame_id    = parent_frame;
  triangles_marker.header.frame_id = parent_frame;
  lines_marker.header.frame_id     = parent_frame;
}

//}

/* dynamicReconfigureCallback //{ */
/* void BatchVisualizer::dynamicReconfigureCallback(mrs_lib::batch_visualizerConfig& config, [[maybe_unused]] uint32_t level) { */
/*   reconfigured = true; */
/*   ROS_INFO("[%s]: Dynamic reconfigure request recieved!", ros::this_node::getName().c_str()); */
/*   tmp_points_scale = config.points_scale; */
/*   tmp_lines_scale  = config.lines_scale; */
/*   ROS_INFO("[%s]: Points scale: %.3f", ros::this_node::getName().c_str(), tmp_points_scale); */
/*   ROS_INFO("[%s]: Lines scale: %.3f", ros::this_node::getName().c_str(), tmp_lines_scale); */
/* } */
//}

/* initialize //{ */
void BatchVisualizer::initialize([[maybe_unused]]ros::NodeHandle& nh) {
  if (initialized) {
    return;
  }

  // setup points marker
  std::stringstream ss;
  ss << marker_topic_name << "_points";
  points_marker.header.frame_id    = parent_frame;
  points_marker.header.stamp       = ros::Time::now();
  points_marker.ns                 = ss.str().c_str();
  points_marker.action             = visualization_msgs::Marker::ADD;
  points_marker.pose.orientation.w = 1.0;
  points_marker.id                 = 8;
  points_marker.type               = visualization_msgs::Marker::POINTS;

  points_marker.scale.x = points_scale;
  points_marker.scale.y = points_scale;

  // setup lines marker
  ss.str(std::string());
  ss << marker_topic_name << "_lines";
  lines_marker.header.frame_id    = parent_frame;
  lines_marker.header.stamp       = ros::Time::now();
  lines_marker.ns                 = ss.str().c_str();
  lines_marker.action             = visualization_msgs::Marker::ADD;
  lines_marker.pose.orientation.w = 1.0;
  lines_marker.id                 = 5;
  lines_marker.type               = visualization_msgs::Marker::LINE_LIST;

  lines_marker.scale.x = lines_scale;

  // setup triangles marker
  ss.str(std::string());
  ss << marker_topic_name << "_triangles";
  triangles_marker.header.frame_id    = parent_frame;
  triangles_marker.header.stamp       = ros::Time::now();
  triangles_marker.ns                 = ss.str().c_str();
  triangles_marker.action             = visualization_msgs::Marker::ADD;
  triangles_marker.pose.orientation.w = 1.0;
  triangles_marker.id                 = 11;
  triangles_marker.type               = visualization_msgs::Marker::TRIANGLE_LIST;

  triangles_marker.scale.x = 1;
  triangles_marker.scale.y = 1;
  triangles_marker.scale.z = 1;

  /* reconfigure_server_.reset(new ReconfigureServer()); */
  /* reconfigure_server_.reset(new ReconfigureServer(nh)); */
  /* ReconfigureServer::CallbackType f = boost::bind(&BatchVisualizer::dynamicReconfigureCallback, this, _1, _2); */
  /* reconfigure_server_->setCallback(f); */

  ROS_INFO("[%s]: Batch visualizer loaded with default values", ros::this_node::getName().c_str());
  initialized = true;
}
//}

/* addPoint //{ */
void BatchVisualizer::addPoint(Eigen::Vector3d p, double r, double g, double b, double a) {
  std_msgs::ColorRGBA color = generateColor(r, g, b, a);
  points_marker.colors.push_back(color);

  geometry_msgs::Point gp = eigenToMsg(p);
  points_marker.points.push_back(gp);
}
//}

/* addRay */  //{
void BatchVisualizer::addRay(mrs_lib::geometry::Ray ray, double r, double g, double b, double a) {

  std_msgs::ColorRGBA color = generateColor(r, g, b, a);

  // color goes in twice, one color for each vertex
  lines_marker.colors.push_back(color);
  lines_marker.colors.push_back(color);

  geometry_msgs::Point p1 = eigenToMsg(ray.p1());
  geometry_msgs::Point p2 = eigenToMsg(ray.p2());

  lines_marker.points.push_back(p1);
  lines_marker.points.push_back(p2);
}
//}

/* addTriangle //{ */
void BatchVisualizer::addTriangle(mrs_lib::geometry::Triangle tri, double r, double g, double b, double a, bool filled) {

  std_msgs::ColorRGBA color = generateColor(r, g, b, a);
  if (filled) {
    // render the triangle face
    // color goes in thrice, one color for each vertex
    triangles_marker.colors.push_back(color);
    triangles_marker.colors.push_back(color);
    triangles_marker.colors.push_back(color);
    triangles_marker.color = color;

    geometry_msgs::Point point1 = eigenToMsg(tri.a());
    geometry_msgs::Point point2 = eigenToMsg(tri.b());
    geometry_msgs::Point point3 = eigenToMsg(tri.c());

    triangles_marker.points.push_back(point1);
    triangles_marker.points.push_back(point2);
    triangles_marker.points.push_back(point3);
  } else {
    // build outline from 3 lines
    geometry_msgs::Point p1 = eigenToMsg(tri.a());
    geometry_msgs::Point p2 = eigenToMsg(tri.b());
    geometry_msgs::Point p3 = eigenToMsg(tri.c());

    lines_marker.colors.push_back(color);
    lines_marker.colors.push_back(color);
    lines_marker.points.push_back(p1);
    lines_marker.points.push_back(p2);

    lines_marker.colors.push_back(color);
    lines_marker.colors.push_back(color);
    lines_marker.points.push_back(p2);
    lines_marker.points.push_back(p3);

    lines_marker.colors.push_back(color);
    lines_marker.colors.push_back(color);
    lines_marker.points.push_back(p3);
    lines_marker.points.push_back(p1);
  }
}
//}

/* addRectangle //{ */
void BatchVisualizer::addRectangle(mrs_lib::geometry::Rectangle rect, double r, double g, double b, double a, bool filled) {
  std::vector<mrs_lib::geometry::Triangle> triangles = rect.triangles();
  addTriangle(triangles[0], r, g, b, a, filled);
  addTriangle(triangles[1], r, g, b, a, filled);
}
//}

/* addCuboid //{ */
void BatchVisualizer::addCuboid(mrs_lib::geometry::Cuboid cuboid, double r, double g, double b, double a, bool filled) {

  for (int i = 0; i < 6; i++) {
    mrs_lib::geometry::Rectangle             rect      = cuboid.getRectangle(i);
    std::vector<mrs_lib::geometry::Triangle> triangles = rect.triangles();
    addTriangle(triangles[0], r, g, b, a, filled);
    addTriangle(triangles[1], r, g, b, a, filled);
  }
}
//}

/* addEllipse //{ */
void BatchVisualizer::addEllipse(mrs_lib::geometry::Ellipse ellipse, double r, double g, double b, double a, bool filled, int num_points) {
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

/* addCylinder //{ */
void BatchVisualizer::addCylinder(mrs_lib::geometry::Cylinder cylinder, double r, double g, double b, double a, bool filled, bool capped, int sides) {
  if (capped) {
    mrs_lib::geometry::Ellipse top    = cylinder.getCap(mrs_lib::geometry::Cylinder::TOP);
    mrs_lib::geometry::Ellipse bottom = cylinder.getCap(mrs_lib::geometry::Cylinder::BOTTOM);
    addEllipse(top, r, g, b, a, filled, sides);
    addEllipse(bottom, r, g, b, a, filled, sides);
  }
  std::vector<Eigen::Vector3d> top_points    = buildEllipse(cylinder.getCap(mrs_lib::geometry::Cylinder::TOP), sides);
  std::vector<Eigen::Vector3d> bottom_points = buildEllipse(cylinder.getCap(mrs_lib::geometry::Cylinder::BOTTOM), sides);
  for (unsigned int i = 0; i < top_points.size() - 1; i++) {
    mrs_lib::geometry::Rectangle rect(bottom_points[i], bottom_points[i + 1], top_points[i + 1], top_points[i]);
    addRectangle(rect, r, g, b, a, filled);
  }
  mrs_lib::geometry::Rectangle rect(bottom_points[bottom_points.size() - 1], bottom_points[0], top_points[0], top_points[top_points.size() - 1]);
  addRectangle(rect, r, g, b, a, filled);
}
//}

/* addCone //{ */
void BatchVisualizer::addCone(mrs_lib::geometry::Cone cone, double r, double g, double b, double a, bool filled, bool capped, int sides) {
  if (capped) {
    mrs_lib::geometry::Ellipse cap = cone.getCap();
    addEllipse(cap, r, g, b, a, filled, sides);
  }
  std::vector<Eigen::Vector3d> cap_points = buildEllipse(cone.getCap(), sides);
  for (unsigned int i = 0; i < cap_points.size() - 1; i++) {
    mrs_lib::geometry::Triangle tri(cap_points[i], cap_points[i + 1], cone.origin());
    addTriangle(tri, r, g, b, a, filled);
  }
  mrs_lib::geometry::Triangle tri(cap_points[cap_points.size() - 1], cap_points[0], cone.origin());
  addTriangle(tri, r, g, b, a, filled);
}
//}

/* addTrajectory //{ */
void BatchVisualizer::addTrajectory(mrs_msgs::TrajectoryReference traj, double r, double g, double b, double a, bool filled) {
  if (traj.points.size() < 2) {
    ROS_WARN("[%s]: Trajectory too short to visualize!", ros::this_node::getName().c_str());
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
    for (size_t i = 0; i < traj.points.size(); i++) {
      Eigen::Vector3d p(traj.points[i].position.x, traj.points[i].position.y, traj.points[i].position.z);
      addPoint(p);
    }
  }
}
//}

/* buildEllipse //{ */
std::vector<Eigen::Vector3d> BatchVisualizer::buildEllipse(mrs_lib::geometry::Ellipse ellipse, int num_points) {
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

/* setPointsScale //{ */
void BatchVisualizer::setPointsScale(double scale) {
  reconfigured = false;
  points_scale = scale;
}
//}

/* setLinesScale //{ */
void BatchVisualizer::setLinesScale(double scale) {
  reconfigured = false;
  lines_scale  = scale;
}
//}

/* clearBuffers //{ */
void BatchVisualizer::clearBuffers() {
  points_marker.points.clear();
  points_marker.colors.clear();

  lines_marker.points.clear();
  lines_marker.colors.clear();

  triangles_marker.points.clear();
  triangles_marker.colors.clear();
}
//}

/* addNullPoint //{ */
void BatchVisualizer::addNullPoint() {
  geometry_msgs::Point p;
  p.x = 10000.0;
  p.y = 0.0;
  p.z = 0.0;

  std_msgs::ColorRGBA c;
  c.r = 1.0;
  c.g = 1.0;
  c.b = 1.0;
  c.a = 1.0;

  points_marker.points.push_back(p);
  points_marker.colors.push_back(c);
}
//}

/* addNullLine //{ */
void BatchVisualizer::addNullLine() {
  geometry_msgs::Point p1, p2;
  p1.x = 10000.0;
  p1.y = 0.0;
  p1.z = 0.0;

  p2.x = 10001.0;
  p2.y = 0.0;
  p2.z = 0.0;

  std_msgs::ColorRGBA c;
  c.r = 1.0;
  c.g = 1.0;
  c.b = 1.0;

  lines_marker.colors.push_back(c);
  lines_marker.colors.push_back(c);

  lines_marker.points.push_back(p1);
  lines_marker.points.push_back(p2);
}
//}

/* addNullTriangle //{ */
void BatchVisualizer::addNullTriangle() {
  geometry_msgs::Point p1, p2, p3;
  p1.x = 10000.0;
  p1.y = 0.0;
  p1.z = 0.0;

  p2.x = 10001.0;
  p2.y = 0.0;
  p2.z = 0.0;

  std_msgs::ColorRGBA c;
  c.r = 1.0;
  c.g = 1.0;
  c.b = 1.0;

  p3.x = 10001.0;
  p3.y = 0.01;
  p3.z = 0.0;
  triangles_marker.colors.push_back(c);
  triangles_marker.colors.push_back(c);
  triangles_marker.colors.push_back(c);

  triangles_marker.points.push_back(p1);
  triangles_marker.points.push_back(p2);
  triangles_marker.points.push_back(p3);
}
//}

/* clearVisuals //{ */
void BatchVisualizer::clearVisuals() {

  visualization_msgs::Marker points_tmp    = points_marker;
  visualization_msgs::Marker lines_tmp     = lines_marker;
  visualization_msgs::Marker triangles_tmp = triangles_marker;

  clearBuffers();
  addNullPoint();
  addNullLine();
  addNullTriangle();

  publish();
  ros::spinOnce();

  points_marker    = points_tmp;
  lines_marker     = lines_tmp;
  triangles_marker = triangles_tmp;
}
//}

/* publish //{ */
void BatchVisualizer::publish() {

  if (reconfigured) {
    setPointsScale(tmp_points_scale);
    setLinesScale(tmp_lines_scale);
  }

  msg.markers.clear();

  bool resolve_empty_points    = points_marker.points.empty();
  bool resolve_empty_lines     = lines_marker.points.empty();
  bool resolve_empty_triangles = triangles_marker.points.empty();

  if (resolve_empty_points) {
    addNullPoint();
  }
  if (resolve_empty_lines) {
    addNullLine();
  }
  if (resolve_empty_triangles) {
    addNullTriangle();
  }

  points_marker.header.stamp = ros::Time::now();
  points_marker.scale.x      = points_scale;
  points_marker.scale.y      = points_scale;
  msg.markers.push_back(points_marker);

  lines_marker.header.stamp = ros::Time::now();
  lines_marker.scale.x      = lines_scale;
  msg.markers.push_back(lines_marker);

  triangles_marker.header.stamp = ros::Time::now();
  msg.markers.push_back(triangles_marker);
  visual_pub.publish(msg);

  if (resolve_empty_points) {
    points_marker.points.clear();
    points_marker.colors.clear();
  }
  if (resolve_empty_lines) {
    lines_marker.points.clear();
    lines_marker.colors.clear();
  }
  if (resolve_empty_triangles) {
    triangles_marker.points.clear();
    triangles_marker.colors.clear();
  }
}
//}

}  // namespace mrs_lib
