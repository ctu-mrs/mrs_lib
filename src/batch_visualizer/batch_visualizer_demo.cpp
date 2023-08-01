#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/batch_visualizer.h>

#include <random>

using namespace mrs_lib;
using namespace mrs_lib::geometry;

double range_min = -20;
double range_max = 20;

std::mt19937                           generator(time(0));
std::uniform_real_distribution<double> rand_dbl(range_min, range_max);
std::uniform_real_distribution<double> rand_percent(0, 1);

int main(int argc, char** argv) {

  ros::init(argc, argv, "batch_visualizer_demo");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ROS_INFO("[%s]: Test started!", ros::this_node::getName().c_str());

  BatchVisualizer bv;
  bv = BatchVisualizer(nh, "test_markers", "map");
  bv.setPointsScale(0.4);
  bv.clearVisuals();

  /* RANDOM POINTS //{ */
  ROS_INFO("[%s]: Generating random points..", ros::this_node::getName().c_str());
  for (int i = 0; (i < 400 && ros::ok()); i++) {
    double          x = rand_dbl(generator);
    double          y = rand_dbl(generator);
    double          z = rand_dbl(generator);
    Eigen::Vector3d point(x, y, z);
    double          r = (x - range_min) / (range_max - range_min);
    double          g = (y - range_min) / (range_max - range_min);
    double          b = (z - range_min) / (range_max - range_min);
    double          a = rand_percent(generator);
    bv.addPoint(point, r, g, b, a, ros::Duration(0.2));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  //}

  /* RANDOM RAYS //{ */
  ROS_INFO("[%s]: Generating random rays...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d point2(x2, y2, z2);
    Ray             ray = Ray::twopointCast(point1, point2);
    double          r   = ((x1 * x2) - range_min) / (range_max - range_min);
    double          g   = ((y1 * y2) - range_min) / (range_max - range_min);
    double          b   = ((z1 * z2) - range_min) / (range_max - range_min);
    double          a   = rand_percent(generator);
    bv.addRay(ray, r, g, b, a, ros::Duration(0.7));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  //}

  /* RAYS FROM ORIGIN //{ */
  ROS_INFO("[%s]: Generating rays originating from one point..", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          a = rand_dbl(generator);
    double          b = rand_dbl(generator);
    double          c = rand_dbl(generator);
    Eigen::Vector3d point(a, b, c);
    Ray             ray = Ray::twopointCast(Eigen::Vector3d::Zero(), point);
    double          r   = (i + 1) / 100.0;
    double          g   = 1 - r;
    bv.addRay(ray, r, g, 0, 1, ros::Duration(0.7));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  //}

  /* RANDOM FILLED TRIANGLES //{ */
  ROS_INFO("[%s]: Generating random filled triangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d point2(x2, y2, z2);
    double          x3 = rand_dbl(generator);
    double          y3 = rand_dbl(generator);
    double          z3 = rand_dbl(generator);
    Eigen::Vector3d point3(x3, y3, z3);
    double          r = x1 * x2 * x3;
    double          g = y1 * y2 * y3;
    double          b = z1 * z2 * z3;
    double          a = rand_percent(generator);
    Triangle        tri(point1, point2, point3);
    bv.addTriangle(tri, r, g, b, a, true, ros::Duration(0.5));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  //}

  /* RANDOM OUTLINED TRIANGLES //{ */
  ROS_INFO("[%s]: Generating random outlined triangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d point2(x2, y2, z2);
    double          x3 = rand_dbl(generator);
    double          y3 = rand_dbl(generator);
    double          z3 = rand_dbl(generator);
    Eigen::Vector3d point3(x3, y3, z3);
    double          r = x1 * x2 * x3;
    double          g = y1 * y2 * y3;
    double          b = z1 * z2 * z3;
    double          a = rand_percent(generator);
    Triangle        tri(point1, point2, point3);
    bv.addTriangle(tri, r, g, b, a, false, ros::Duration(0.5));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  //}

  /* RANDOM RECTANGLES //{ */
  ROS_INFO("[%s]: Generating random rectangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d point2(x2, y2, z2);
    double          plane_x = rand_dbl(generator);
    double          plane_y = rand_dbl(generator);
    double          plane_z = rand_dbl(generator);
    Eigen::Vector3d plane_anchor(plane_x, plane_y, plane_z);
    double          side_length2 = rand_dbl(generator) / 2.0;
    Eigen::Vector3d point3       = point2 + side_length2 * (((point2 - point1).cross(plane_anchor - point1)).normalized());
    Eigen::Vector3d point4       = point1 + side_length2 * (((point2 - point1).cross(plane_anchor - point1)).normalized());

    double    r = x1 * x2;
    double    g = y1 * y2;
    double    b = z1 * z2;
    double    a = rand_percent(generator);
    Rectangle rect(point1, point2, point3, point4);
    bv.addRectangle(rect, r, g, b, a, true, ros::Duration(0.5));   // draw colored faces
    bv.addRectangle(rect, 0, 0, 0, 1, false, ros::Duration(0.7));  // draw outlines
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  bv.clearBuffers();
  //}

  /* RANDOM CUBES //{ */
  ROS_INFO("[%s]: Generating random cubes...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double             x1 = rand_dbl(generator);
    double             y1 = rand_dbl(generator);
    double             z1 = rand_dbl(generator);
    Eigen::Vector3d    center(x1, y1, z1);
    double             s = 0.4 * rand_dbl(generator);
    Eigen::Vector3d    scale(s, s, s);
    double             x2          = rand_dbl(generator);
    double             y2          = rand_dbl(generator);
    double             z2          = rand_dbl(generator);
    Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

    double r = (x1 - range_min) / (range_max - range_min);
    double g = (y1 - range_min) / (range_max - range_min);
    double b = (z1 - range_min) / (range_max - range_min);
    double a = rand_percent(generator);
    Cuboid cub(center, scale, orientation);

    bv.addCuboid(cub, r, g, b, a, true, ros::Duration(0.3));   // draw colored faces
    bv.addCuboid(cub, 0, 0, 0, 1, false, ros::Duration(0.3));  // draw outlines
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RANDOM ELLIPSES //{ */
  ROS_INFO("[%s]: Generating random ellipses...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double             x1 = rand_dbl(generator);
    double             y1 = rand_dbl(generator);
    double             z1 = rand_dbl(generator);
    Eigen::Vector3d    center(x1, y1, z1);
    double             major       = 0.4 * rand_dbl(generator);
    double             minor       = 0.1 * rand_dbl(generator);
    double             x2          = rand_dbl(generator);
    double             y2          = rand_dbl(generator);
    double             z2          = rand_dbl(generator);
    Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

    double  r = (x1 - range_min) / (range_max - range_min);
    double  g = (y1 - range_min) / (range_max - range_min);
    double  b = (z1 - range_min) / (range_max - range_min);
    double  a = rand_percent(generator);
    Ellipse el(center, orientation, major, minor);

    bv.addEllipse(el, r, g, b, a, true, 64, ros::Duration(0.5));           // colored face
    bv.addEllipse(el, 0.0, 0.0, 0.0, 1.0, false, 64, ros::Duration(0.5));  // black outline
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RANDOM CYLINDERS//{ */
  ROS_INFO("[%s]: Generating random cylinders...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double             x1 = rand_dbl(generator);
    double             y1 = rand_dbl(generator);
    double             z1 = rand_dbl(generator);
    Eigen::Vector3d    center(x1, y1, z1);
    double             x2          = rand_dbl(generator);
    double             y2          = rand_dbl(generator);
    double             z2          = rand_dbl(generator);
    Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

    double radius = 0.3 * rand_dbl(generator);
    double height = rand_dbl(generator);

    double r = (x1 - range_min) / (range_max - range_min);
    double g = (y1 - range_min) / (range_max - range_min);
    double b = (z1 - range_min) / (range_max - range_min);
    double a = rand_percent(generator);

    Cylinder cyl(center, radius, height, orientation);
    bv.addCylinder(cyl, r, g, b, a, true, true, 12, ros::Duration(0.5));
    bv.addCylinder(cyl, 0, 0, 0, 1, false, false, 12, ros::Duration(0.5));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RANDOM CONES //{ */
  ROS_INFO("[%s]: Generating random cones...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d origin(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d direction(x2, y2, z2);

    double angle  = 0.02 * rand_dbl(generator);
    double height = rand_dbl(generator);

    double r = (x1 - range_min) / (range_max - range_min);
    double g = (y1 - range_min) / (range_max - range_min);
    double b = (z1 - range_min) / (range_max - range_min);
    double a = rand_percent(generator);

    Cone cone(origin, angle, height, direction);
    bv.addCone(cone, r, g, b, a, true, true, 12, ros::Duration(0.5));
    bv.addCone(cone, 0, 0, 0, 1, false, false, 12, ros::Duration(0.5));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RANDOM TRAJECTORIES //{ */
  ROS_INFO("[%s]: Generating random trajectories...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 200 && ros::ok()); i++) {
    mrs_msgs::TrajectoryReference traj;
    for (int n = 0; n < 10; n++) {
      double              x = rand_dbl(generator);
      double              y = rand_dbl(generator);
      double              z = rand_dbl(generator);
      mrs_msgs::Reference ref;
      ref.position.x = x;
      ref.position.y = y;
      ref.position.z = z;
      traj.points.push_back(ref);
    }

    bv.addTrajectory(traj, 0, 1, 1, 1, true, ros::Duration(0.08));
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  bv.clearBuffers();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RAYTRACING TEST //{ */
  bv.setLinesScale(0.3);
  bv.setPointsScale(1.0);
  ROS_INFO("[%s]: Generating random rays and triangles...", ros::this_node::getName().c_str());
  for (int i = 0; (i < 20 && ros::ok()); i++) {
    double          x1 = rand_dbl(generator);
    double          y1 = rand_dbl(generator);
    double          z1 = rand_dbl(generator);
    Eigen::Vector3d point1(x1, y1, z1);
    double          x2 = rand_dbl(generator);
    double          y2 = rand_dbl(generator);
    double          z2 = rand_dbl(generator);
    Eigen::Vector3d point2(x2, y2, z2);
    double          x3 = rand_dbl(generator);
    double          y3 = rand_dbl(generator);
    double          z3 = rand_dbl(generator);
    Eigen::Vector3d point3(3 * x3, 3 * y3, 3 * z3);
    Triangle        tri(point1, point2, point3);

    double          rx1 = rand_dbl(generator);
    double          ry1 = rand_dbl(generator);
    double          rz1 = rand_dbl(generator);
    Eigen::Vector3d ray_point1(4 * rx1, 4 * ry1, 4 * rz1);
    double          rx2 = rand_dbl(generator);
    double          ry2 = rand_dbl(generator);
    double          rz2 = rand_dbl(generator);
    Eigen::Vector3d ray_point2(4 * rx2, 4 * ry2, 4 * rz2);
    Ray             ray = Ray::twopointCast(ray_point1, ray_point2);
    bv.addRay(ray, 1.0, 0.5, 0.0, 1.0, ros::Duration(0.3));

    if (tri.intersectionRay(ray) == boost::none) {
      bv.addTriangle(tri, 1.0, 0.0, 0.0, 1.0, true, ros::Duration(0.3));
    } else {
      bv.addTriangle(tri, 0.0, 1.0, 0.3, 1.0, true, ros::Duration(0.3));
      bv.addPoint(tri.intersectionRay(ray).get(), 1, 1, 0, 1, ros::Duration(0.3));
    }
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.6).sleep();
  }
  bv.clearBuffers();
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  /* RANDOM CHAOS //{ */
  bv.setLinesScale(0.08);
  bv.setPointsScale(0.6);
  ROS_INFO("[%s]: Generating random chaos...", ros::this_node::getName().c_str());
  for (int n = 0; n < 100; n++) {
    for (int i = 0; (i < 300 && ros::ok()); i++) {
      double          x = rand_dbl(generator);
      double          y = rand_dbl(generator);
      double          z = rand_dbl(generator);
      Eigen::Vector3d point(x, y, z);
      double          r           = (x - range_min) / (range_max - range_min);
      double          g           = (y - range_min) / (range_max - range_min);
      double          b           = (z - range_min) / (range_max - range_min);
      double          a           = rand_percent(generator);
      double          timeout_sec = 0.2 * rand_percent(generator);
      bv.addPoint(point, r, g, b, a, ros::Duration(timeout_sec));
    }
    for (int i = 0; (i < 100 && ros::ok()); i++) {
      double             x1 = rand_dbl(generator);
      double             y1 = rand_dbl(generator);
      double             z1 = rand_dbl(generator);
      Eigen::Vector3d    center(x1, y1, z1);
      double             s = 0.4 * rand_dbl(generator);
      Eigen::Vector3d    scale(s, s, s);
      double             x2          = rand_dbl(generator);
      double             y2          = rand_dbl(generator);
      double             z2          = rand_dbl(generator);
      Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

      double r = (x1 - range_min) / (range_max - range_min);
      double g = (y1 - range_min) / (range_max - range_min);
      double b = (z1 - range_min) / (range_max - range_min);
      double a = rand_percent(generator);
      Cuboid cub(center, scale, orientation);
      double timeout_sec = 0.2 * rand_percent(generator);

      bv.addCuboid(cub, r, g, b, a, true, ros::Duration(timeout_sec));   // draw colored faces
      bv.addCuboid(cub, 0, 0, 0, 1, false, ros::Duration(timeout_sec));  // draw outlines
    }
    for (int i = 0; (i < 100 && ros::ok()); i++) {
      double             x1 = rand_dbl(generator);
      double             y1 = rand_dbl(generator);
      double             z1 = rand_dbl(generator);
      Eigen::Vector3d    center(x1, y1, z1);
      double             x2          = rand_dbl(generator);
      double             y2          = rand_dbl(generator);
      double             z2          = rand_dbl(generator);
      Eigen::Quaterniond orientation = mrs_lib::geometry::quaternionFromEuler(x2, y2, z2);

      double radius = 0.3 * rand_dbl(generator);
      double height = rand_dbl(generator);

      double r = (x1 - range_min) / (range_max - range_min);
      double g = (y1 - range_min) / (range_max - range_min);
      double b = (z1 - range_min) / (range_max - range_min);
      double a = rand_percent(generator);

      Cylinder cyl(center, radius, height, orientation);
      double   timeout_sec = 0.2 * rand_percent(generator);
      bv.addCylinder(cyl, r, g, b, a, true, true, 12, ros::Duration(timeout_sec));
      bv.addCylinder(cyl, 0, 0, 0, 1, false, false, 12, ros::Duration(timeout_sec));
    }
    bv.publish();
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  ROS_INFO("[%s]: Done", ros::this_node::getName().c_str());
  //}

  ROS_INFO("[%s]: TEST DONE!", ros::this_node::getName().c_str());
  return 0;
}
