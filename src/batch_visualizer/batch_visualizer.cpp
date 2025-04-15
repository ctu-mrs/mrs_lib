#include <string>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/geometry/shapes.h>

namespace mrs_lib
{

/* constructors */  //{
BatchVisualizer::BatchVisualizer() {
}

BatchVisualizer::~BatchVisualizer() {
}

BatchVisualizer::BatchVisualizer(const std::shared_ptr<rclcpp::Node> &node, const std::string marker_topic_name, const std::string parent_frame) {
  this->node              = node;
  this->parent_frame      = parent_frame;
  this->marker_topic_name = marker_topic_name;
  initialize();

  this->visual_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_name, rclcpp::SystemDefaultsQoS());
  publish();
}
//}

/* setParentFrame //{ */

void BatchVisualizer::setParentFrame(const std::string parent_frame) {
  this->parent_frame               = parent_frame;
  points_marker.header.frame_id    = parent_frame;
  triangles_marker.header.frame_id = parent_frame;
  lines_marker.header.frame_id     = parent_frame;
}

//}

/* initialize //{ */
void BatchVisualizer::initialize() {
  if (initialized) {
    return;
  }

  // setup points marker
  std::stringstream ss;
  ss << marker_topic_name << "_points";
  points_marker.header.frame_id    = parent_frame;
  points_marker.header.stamp       = node->get_clock()->now();
  points_marker.ns                 = ss.str().c_str();
  points_marker.action             = visualization_msgs::msg::Marker::ADD;
  points_marker.pose.orientation.w = 1.0;
  points_marker.id                 = 8;
  points_marker.type               = visualization_msgs::msg::Marker::POINTS;

  points_marker.scale.x = points_scale;
  points_marker.scale.y = points_scale;
  points_marker.color.a = 1.0;

  // setup lines marker
  ss.str(std::string());
  ss << marker_topic_name << "_lines";
  lines_marker.header.frame_id    = parent_frame;
  lines_marker.header.stamp       = node->get_clock()->now();
  lines_marker.ns                 = ss.str().c_str();
  lines_marker.action             = visualization_msgs::msg::Marker::ADD;
  lines_marker.pose.orientation.w = 1.0;
  lines_marker.id                 = 5;
  lines_marker.type               = visualization_msgs::msg::Marker::LINE_LIST;

  lines_marker.scale.x = lines_scale;
  lines_marker.color.a = 1.0;

  // setup triangles marker
  ss.str(std::string());
  ss << marker_topic_name << "_triangles";
  triangles_marker.header.frame_id    = parent_frame;
  triangles_marker.header.stamp       = node->get_clock()->now();
  triangles_marker.ns                 = ss.str().c_str();
  triangles_marker.action             = visualization_msgs::msg::Marker::ADD;
  triangles_marker.pose.orientation.w = 1.0;
  triangles_marker.id                 = 11;
  triangles_marker.type               = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  triangles_marker.scale.x = 1;
  triangles_marker.scale.y = 1;
  triangles_marker.scale.z = 1;
  triangles_marker.color.a = 1.0;

  RCLCPP_INFO(node->get_logger(), "[%s]: Batch visualizer loaded with default values", node->get_name());
  initialized = true;
}
//}

/* addPoint //{ */
void BatchVisualizer::addPoint(const Eigen::Vector3d &p, const double r, const double g, const double b, const double a, const rclcpp::Duration &timeout) {

  VisualObject obj = VisualObject(p, r, g, b, a, timeout, uuid++, this->node);
  visual_objects.insert(obj);
}
//}

/* addRay */  //{
void BatchVisualizer::addRay(const mrs_lib::geometry::Ray &ray, const double r, const double g, const double b, const double a, const rclcpp::Duration &timeout) {

  VisualObject obj = VisualObject(ray, r, g, b, a, timeout, uuid++, this->node);
  visual_objects.insert(obj);
}
//}

/* addTriangle //{ */
void BatchVisualizer::addTriangle(const mrs_lib::geometry::Triangle &tri, const double r, const double g, const double b, const double a, const bool filled, const rclcpp::Duration &timeout) {

  VisualObject obj = VisualObject(tri, r, g, b, a, timeout, filled, uuid++, this->node);
  visual_objects.insert(obj);
}
//}

/* addRectangle //{ */
void BatchVisualizer::addRectangle(const mrs_lib::geometry::Rectangle &rect, const double r, const double g, const double b, const double a, const bool filled, const rclcpp::Duration &timeout) {

  VisualObject obj = VisualObject(rect, r, g, b, a, timeout, filled, uuid++, this->node);
  visual_objects.insert(obj);
}
//}

/* addCuboid //{ */
void BatchVisualizer::addCuboid(const mrs_lib::geometry::Cuboid &cuboid, const double r, const double g, const double b, const double a, const bool filled, const rclcpp::Duration &timeout) {

  VisualObject obj = VisualObject(cuboid, r, g, b, a, timeout, filled, uuid++, this->node);
  visual_objects.insert(obj);
}
//}

/* addEllipse //{ */
void BatchVisualizer::addEllipse(const mrs_lib::geometry::Ellipse &ellipse, const double r, const double g, const double b, const double a, const bool filled, const int num_points, const rclcpp::Duration &timeout) {

  VisualObject obj = VisualObject(ellipse, r, g, b, a, timeout, filled, uuid++, this->node, num_points);
  visual_objects.insert(obj);
}
//}

/* addCylinder //{ */
void BatchVisualizer::addCylinder(const mrs_lib::geometry::Cylinder &cylinder, const double r, const double g, const double b, const double a, const bool filled, const bool capped, const int sides, const rclcpp::Duration &timeout) {
  VisualObject obj = VisualObject(cylinder, r, g, b, a, timeout, filled, capped, uuid++, this->node, sides);
  visual_objects.insert(obj);
}
//}

/* addCone //{ */
void BatchVisualizer::addCone(const mrs_lib::geometry::Cone &cone, const double r, const double g, const double b, const double a, const bool filled, const bool capped, const int sides, const rclcpp::Duration &timeout) {
  VisualObject obj = VisualObject(cone, r, g, b, a, timeout, filled, capped, uuid++, this->node, sides);
  visual_objects.insert(obj);
}
//}

/* addPath //{ */
void BatchVisualizer::addPath(const mrs_msgs::msg::Path &p, const double r, const double g, const double b, const double a, const bool filled, const rclcpp::Duration &timeout) {
  VisualObject obj = VisualObject(p, r, g, b, a, timeout, filled, uuid++, this->node);
  visual_objects.insert(obj);
}
//}

/* addTrajectory //{ */
void BatchVisualizer::addTrajectory(const mrs_msgs::msg::TrajectoryReference &traj, const double r, const double g, const double b, const double a, const bool filled, const rclcpp::Duration &timeout) {
  VisualObject obj = VisualObject(traj, r, g, b, a, timeout, filled, uuid++, this->node);
  visual_objects.insert(obj);
}
//}

/* addNullPoint //{ */
void BatchVisualizer::addNullPoint() {
  geometry_msgs::msg::Point p;
  p.x = 10000.0;
  p.y = 0.0;
  p.z = 0.0;

  std_msgs::msg::ColorRGBA c;
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
  geometry_msgs::msg::Point p1, p2;
  p1.x = 10000.0;
  p1.y = 0.0;
  p1.z = 0.0;

  p2.x = 10001.0;
  p2.y = 0.0;
  p2.z = 0.0;

  std_msgs::msg::ColorRGBA c;
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
  geometry_msgs::msg::Point p1, p2, p3;
  p1.x = 10000.0;
  p1.y = 0.0;
  p1.z = 0.0;

  p2.x = 10001.0;
  p2.y = 0.0;
  p2.z = 0.0;

  std_msgs::msg::ColorRGBA c;
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

/* setPointsScale //{ */
void BatchVisualizer::setPointsScale(const double scale) {
  points_scale = scale;
}
//}

/* setLinesScale //{ */
void BatchVisualizer::setLinesScale(const double scale) {
  lines_scale = scale;
}
//}

/* clearBuffers //{ */
void BatchVisualizer::clearBuffers() {
  visual_objects.clear();
}
//}

/* clearVisuals //{ */
void BatchVisualizer::clearVisuals() {
  std::set<VisualObject> visual_objects_tmp;
  visual_objects_tmp.insert(visual_objects.begin(), visual_objects.end());

  visual_objects.clear();
  publish();

  visual_objects.insert(visual_objects_tmp.begin(), visual_objects_tmp.end());
}
//}

/* publish //{ */
void BatchVisualizer::publish() {

  this->publish(node->get_clock()->now());
}

void BatchVisualizer::publish(const rclcpp::Time stamp) {

  msg.markers.clear();
  points_marker.points.clear();
  points_marker.colors.clear();

  lines_marker.points.clear();
  lines_marker.colors.clear();

  triangles_marker.points.clear();
  triangles_marker.colors.clear();

  // fill marker messages and remove objects that have timed out
  for (auto it = visual_objects.begin(); it != visual_objects.end();) {

    if (it->isTimedOut()) {
      it = visual_objects.erase(it);
    } else {
      auto points = it->getPoints();
      auto colors = it->getColors();
      switch (it->getType()) {
        case MarkerType::POINT: {
          points_marker.points.insert(points_marker.points.end(), points.begin(), points.end());
          points_marker.colors.insert(points_marker.colors.end(), colors.begin(), colors.end());
          break;
        }
        case MarkerType::LINE: {
          lines_marker.points.insert(lines_marker.points.end(), points.begin(), points.end());
          lines_marker.colors.insert(lines_marker.colors.end(), colors.begin(), colors.end());
          break;
        }
        case MarkerType::TRIANGLE: {
          triangles_marker.points.insert(triangles_marker.points.end(), points.begin(), points.end());
          triangles_marker.colors.insert(triangles_marker.colors.end(), colors.begin(), colors.end());
          break;
        }
      }
      it++;
    }
  }

  if (!points_marker.points.empty()) {
    points_marker.scale.x = points_scale;
    points_marker.scale.y = points_scale;
  } else {
    addNullPoint();
  }
  points_marker.header.stamp = stamp;
  msg.markers.push_back(points_marker);

  if (!lines_marker.points.empty()) {
    lines_marker.scale.x = lines_scale;
  } else {
    addNullLine();
  }
  lines_marker.header.stamp = stamp;
  msg.markers.push_back(lines_marker);

  if (!triangles_marker.points.empty()) {
    triangles_marker.header.stamp = stamp;
  } else {
    addNullTriangle();
  }
  triangles_marker.header.stamp = stamp;
  msg.markers.push_back(triangles_marker);

  if (msg.markers.empty()) {
    addNullPoint();
    points_marker.scale.x = 0.1;
    points_marker.scale.y = 0.1;
    msg.markers.push_back(points_marker);
  }

  visual_pub->publish(msg);
}
//}

}  // namespace mrs_lib
