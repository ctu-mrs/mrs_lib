/**  \file
     \brief For convenient and computationally lightweight drawing of geometry in RVIZ using marker arrays.
     \author Petr Štibinger - stibipet@fel.cvut.cz
 */

#ifndef BATCH_VISUALIZER_H
#define BATCH_VISUALIZER_H

#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_lib/geometry/shapes.h>
/* #include <dynamic_reconfigure/server.h> */
/* #include <mrs_lib/batch_visualizerConfig.h> */

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
   * @param nh ROS node to connect our publisher to ROS
   * @param marker_topic_name name of the topic on which the markers will be published
   * @param parent_frame name of the frame to which the markers will be linked
   */
  BatchVisualizer(ros::NodeHandle &nh, std::string marker_topic_name, std::string parent_frame);

  /**
   * @brief add a point to the buffer
   *
   * @param point coordinates of the point
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   */
  void addPoint(Eigen::Vector3d point, double r = 0.0, double g = 1.0, double b = 0.3, double a = 1.0);

  /**
   * @brief add a ray to the buffer
   *
   * @param ray ray to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   */
  void addRay(mrs_lib::geometry::Ray ray, double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0);

  /**
   * @brief add a triangle to the buffer
   *
   * @param tri triangle to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   */
  void addTriangle(mrs_lib::geometry::Triangle tri, double r = 0.5, double g = 0.5, double b = 0.0, double a = 1.0, bool filled = true);

  /**
   * @brief add a rectangle to the buffer
   *
   * @param rect rectangle to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   */
  void addRectangle(mrs_lib::geometry::Rectangle rect, double r = 0.5, double g = 0.5, double b = 0.0, double a = 1.0, bool filled = true);

  /**
   * @brief add a cuboid to the buffer
   *
   * @param cuboid cuboid to be added
   * @param r red color in range <0,1>
   * @param g green color in range <0,1>
   * @param b blue color in range <0,1>
   * @param a alpha in range <0,1> (0 is fully transparent)
   * @param filled bool to set fill. True = face visible, False = outline visible
   */
  void addCuboid(mrs_lib::geometry::Cuboid cuboid, double r = 0.5, double g = 0.5, double b = 0.0, double a = 1.0, bool filled = true);

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
   */
  void addEllipse(mrs_lib::geometry::Ellipse ellipse, double r = 0.0, double g = 1.0, double b = 1.0, double a = 1.0, bool filled = true,
                  int num_points = DEFAULT_ELLIPSE_POINTS);

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
   */
  void addCylinder(mrs_lib::geometry::Cylinder cylinder, double r = 0.7, double g = 0.8, double b = 0.3, double a = 1.0, bool filled = true, bool capped = true,
                   int sides = DEFAULT_ELLIPSE_POINTS);
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
   */
  void addCone(mrs_lib::geometry::Cone cone, double r = 0.7, double g = 0.8, double b = 0.3, double a = 1.0, bool filled = true, bool capped = true,
               int sides = DEFAULT_ELLIPSE_POINTS);

  /**
   * @brief set the scale of all points
   *
   * @param scale
   */
  void setPointsScale(double scale);

  /**
   * @brief set the thickness of all lines
   *
   * @param scale
   */
  void setLinesScale(double scale);

  /**
   * @brief remove all objects from the buffer
   */
  void clearBuffers();

  /**
   * @brief publishes an empty message. Removes all objects drawn onto the scene, but keeps them in buffer
   */
  void clearVisuals();

  /**
   * @brief publish the visual markers ROS message and populates it with buffer content
   */
  void publish();

private:
  ros::Publisher                  visual_pub;
  visualization_msgs::MarkerArray msg;

  std::string parent_frame;
  std::string marker_topic_name;

  visualization_msgs::Marker points_marker;
  visualization_msgs::Marker lines_marker;
  visualization_msgs::Marker triangles_marker;

  bool initialized = false;
  void initialize(ros::NodeHandle &nh);

  double points_scale = 0.02;
  double lines_scale  = 0.04;

  void addNullPoint();
  void addNullLine();
  void addNullTriangle();

  std::vector<Eigen::Vector3d> buildEllipse(mrs_lib::geometry::Ellipse ellispe, int num_points = DEFAULT_ELLIPSE_POINTS);

  // dynamic reconfigure
  /* typedef mrs_lib::batch_visualizerConfig Config; */

  /* typedef dynamic_reconfigure::Server<Config> ReconfigureServer; */
  /* boost::shared_ptr<ReconfigureServer>        reconfigure_server_; */

  /* void dynamicReconfigureCallback(Config &config, uint32_t level); */
};

}  // namespace mrs_lib

#endif
