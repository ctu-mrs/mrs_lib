#ifndef BATCH_VISUALIZER_H
#define BATCH_VISUALIZER_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mrs_lib/geometry/shapes.h>
#include <mrs_msgs/msg/path.hpp>
#include <mrs_msgs/msg/trajectory_reference.hpp>
#include <mrs_lib/visual_object.h>
#include <set>

#define DEFAULT_ELLIPSE_POINTS 64

namespace mrs_lib
{

/**
 * @brief library for drawing large amounts of geometric objects in RVIZ at the same time
 */
class BatchVisualizer {

public:
  /**
   * @brief dummy constructor
   */
  BatchVisualizer();
  /**
   * @brief destructor
   */
  ~BatchVisualizer();
  /**
   * @brief constructor to initialize the visualizer
   *
   * @param node rclcpp node pointer to connect our publisher to rclcpp
   * @param marker_topic_name name of the topic on which the markers will be published
   * @param parent_frame name of the frame to which the markers will be linked
   */
  BatchVisualizer(const std::shared_ptr<rclcpp::Node>& node, std::string marker_topic_name, std::string parent_frame);

  /**
   * @brief add a point to the buffer
   *
   * @param point coordinates of the point
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addPoint(const Eigen::Vector3d &point, const double r = 0.0, const double g = 1.0, const double b = 0.3, const double a = 1.0,
                const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add a ray to the buffer
   *
   * @param ray ray to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addRay(const mrs_lib::geometry::Ray &ray, const double r = 1.0, const double g = 0.0, const double b = 0.0, const double a = 1.0,
              const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add a triangle to the buffer
   *
   * @param tri triangle to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addTriangle(const mrs_lib::geometry::Triangle &tri, const double r = 0.5, const double g = 0.5, const double b = 0.0, const double a = 1.0,
                   const bool filled = true, const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add a rectangle to the buffer
   *
   * @param rect rectangle to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addRectangle(const mrs_lib::geometry::Rectangle &rect, const double r = 0.5, const double g = 0.5, const double b = 0.0, const double a = 1.0,
                    const bool filled = true, const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add a cuboid to the buffer
   *
   * @param cuboid cuboid to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addCuboid(const mrs_lib::geometry::Cuboid &cuboid, const double r = 0.5, const double g = 0.5, const double b = 0.0, const double a = 1.0,
                 const bool filled = true, const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add an ellipse to the buffer
   *
   * @param ellipse ellipse to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   * @param num_points number of points to approximate the round shape
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addEllipse(const mrs_lib::geometry::Ellipse &ellipse, const double r = 0.0, const double g = 1.0, const double b = 1.0, const double a = 1.0,
                  const bool filled = true, const int num_points = DEFAULT_ELLIPSE_POINTS, const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add a cylinder to the buffer
   *
   * @param cylinder cylinder to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   * @param capped bool to set caps on/off. True = caps drawn, False = hollow cylinder
   * @param sides number of points to approximate the round shape
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addCylinder(const mrs_lib::geometry::Cylinder &cylinder, const double r = 0.7, const double g = 0.8, const double b = 0.3, const double a = 1.0,
                   const bool filled = true, const bool capped = true, const int sides = DEFAULT_ELLIPSE_POINTS,
                   const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));
  /**
   * @brief add a cone to the buffer
   *
   * @param cone cone to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   * @param capped bool to set caps on/off. True = cap drawn, False = base cap missing
   * @param sides number of points to approximate the round shape
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addCone(const mrs_lib::geometry::Cone &cone, const double r = 0.7, const double g = 0.8, const double b = 0.3, const double a = 1.0,
               const bool filled = true, const bool capped = true, const int sides = DEFAULT_ELLIPSE_POINTS, const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add a path to the buffer
   *
   * @param p path to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = continuous line, False = only visualize points
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addPath(const mrs_msgs::msg::Path &p, const double r = 0.3, const double g = 1.0, const double b = 0.3, const double a = 1.0, const bool filled = true,
               const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief add a trajectory to the buffer
   *
   * @param traj trajectory reference to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = continuous line, False = only visualize points
   * @param timeout time in seconds after which the object should be removed from buffer
   */
  void addTrajectory(const mrs_msgs::msg::TrajectoryReference &traj, const double r = 0.3, const double g = 1.0, const double b = 0.3, const double a = 1.0,
                     const bool filled = true, const rclcpp::Duration &timeout = rclcpp::Duration(std::chrono::seconds(0)));

  /**
   * @brief helper function for adding an invisible point to the object buffer
   */
  void addNullPoint();

  /**
   * @brief helper function for adding an invisible line to the object buffer
   */
  void addNullLine();

  /**
   * @brief helper function for adding an invisible triangle to the buffer
   */
  void addNullTriangle();

  /**
   * @brief set the scale of all points
   *
   * @param scale
   */
  void setPointsScale(const double scale);

  /**
   * @brief set the parent frame_id
   *
   * @param parent_frame
   */
  void setParentFrame(const std::string parent_frame);

  /**
   * @brief set the thickness of all lines
   *
   * @param scale
   */
  void setLinesScale(const double scale);

  /**
   * @brief remove all objects from the buffer
   */
  void clearBuffers();

  /**
   * @brief publishes an empty message. Removes all objects drawn onto the scene, but keeps them in buffer
   */
  void clearVisuals();

  /**
   * @brief publish the visual markers rclcpp message and populates it with buffer content
   */
  void publish();

  /**
   * @brief publish the visual markers rclcpp message with desired stamp and populates it with buffer content
   */
  void publish(const rclcpp::Time stamp);

private:
  std::shared_ptr<rclcpp::Node>   node;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr                  visual_pub;
  visualization_msgs::msg::MarkerArray msg;

  std::string parent_frame;  // coordinate frame id
  std::string marker_topic_name;

  std::set<VisualObject> visual_objects;  // buffer for objects to be visualized

  visualization_msgs::msg::Marker points_marker;
  visualization_msgs::msg::Marker lines_marker;
  visualization_msgs::msg::Marker triangles_marker;

  bool initialized = false;
  void initialize();

  double points_scale = 0.02;
  double lines_scale  = 0.04;

  unsigned long uuid = 0;  // create unique ID for items in object buffer
};

}  // namespace mrs_lib

#endif
