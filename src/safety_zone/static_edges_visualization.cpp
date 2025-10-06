#include "mrs_lib/safety_zone/static_edges_visualization.h"
#include <mrs_lib/attitude_converter.h>
#include <geometry_msgs/msg/point.h>
#include <atomic>

namespace mrs_lib
{

  std::atomic<int> StaticEdgesVisualization::id_generator_ = 0;
  visualization_msgs::msg::MarkerArray StaticEdgesVisualization::last_markers_ = visualization_msgs::msg::MarkerArray();

  /* StaticEdgesVisualization(prism) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(safety_zone::Prism prism, const std::string& uav_name, const std::string& frame_id, const rclcpp::Node::SharedPtr node,
                                                     const double& markers_update_rate)
      : id_(id_generator_++), prism_(prism), uav_name_(uav_name), frame_id_(frame_id), node_(node)
  {

    timer_= node_->create_wall_timer(std::chrono::duration<double>(markers_update_rate),[this](){ sendMarker(); }); 
    last_markers_.markers.push_back(visualization_msgs::msg::Marker());
    init();
  }

  //}

  /* StaticEdgesVisualization(obstacle) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(safety_zone::SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name, const std::string& frame_id,
                                                     const rclcpp::Node::SharedPtr node, const double& markers_update_rate)

      : id_(id_generator_++), prism_(safety_zone->getObstacle(obstacle_id)), uav_name_(uav_name), frame_id_(frame_id), node_(node)

  {
    timer_ = node_->create_wall_timer(std::chrono::duration<double>(markers_update_rate),[this](){ sendMarker();});
    last_markers_.markers.push_back(visualization_msgs::msg::Marker());
    init();
  }

  //}

  /* StaticEdgesVisualization(safety border) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(safety_zone::SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id,
                                                     const rclcpp::Node::SharedPtr node, const double& markers_update_rate)

      : id_(id_generator_++), prism_(safety_zone->getBorder()), uav_name_(uav_name), frame_id_(frame_id), node_(node)
  {

    last_markers_.markers.push_back(visualization_msgs::msg::Marker());

    timer_ = node_->create_wall_timer(std::chrono::duration<double>(markers_update_rate),[this](){ sendMarker();});

    init();
  }

  //}

  /* ~StaticEdgesVisualization() //{ */

  StaticEdgesVisualization::~StaticEdgesVisualization()
  {
    if (is_active_)
    {
      prism_.unsubscribe(this);
    }
    cleanup();
  }

  //}

  /* init() //{ */

  void StaticEdgesVisualization::init()
  {

    publisher_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(node_, "safety_area_static_markers_out"); 
    prism_.subscribe(this);
    update();
    is_active_ = true;  // is_active_ flag inherited from Subscriber class defined in safety_zone::Prism
  }

  //}

  /* sendMarker() //{ */

  // Lines must be updated periodically. View https://github.com/ros-visualization/rviz/issues/1287
  void StaticEdgesVisualization::sendMarker()
  {
    publisher_.publish(last_markers_);
  }

  //}

  /* update() //{ */

  void StaticEdgesVisualization::update()
  {
    if (!is_active_)
    {
      return;
    }

    const double max_z = prism_.getMaxZ();
    const double min_z = prism_.getMinZ();
    const auto polygon = prism_.getPolygon().outer();

    visualization_msgs::msg::Marker marker;
    marker.id = id_;
    std::string target_frame_id = "world_origin";
    marker.header.frame_id = uav_name_ + "/" + target_frame_id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    marker.scale.x = 0.2;

    // Iterating over polygon edges to publish the marker lines

    for (size_t i = 0; i < polygon.size() - 1; i++)
    {  // -1 because the last is the same as the first one
      // Adding upper horizontal edges
      geometry_msgs::msg::Point upper_start;
      geometry_msgs::msg::Point upper_end;

      upper_start.x = polygon[i].get<0>();
      upper_start.y = polygon[i].get<1>();
      upper_start.z = max_z;

      upper_end.x = polygon[i + 1].get<0>();
      upper_end.y = polygon[i + 1].get<1>();
      upper_end.z = max_z;

      marker.points.push_back(upper_start);

      marker.points.push_back(upper_end);

      // Adding lower horizontal edges
      geometry_msgs::msg::Point lower_start;
      geometry_msgs::msg::Point lower_end;

      lower_start.x = polygon[i].get<0>();
      lower_start.y = polygon[i].get<1>();
      lower_start.z = min_z;

      lower_end.x = polygon[i + 1].get<0>();
      lower_end.y = polygon[i + 1].get<1>();
      lower_end.z = min_z;

      marker.points.push_back(lower_start);

      marker.points.push_back(lower_end);

      // Adding vertical edges
      marker.points.push_back(upper_start);
      marker.points.push_back(lower_start);
    }

    last_markers_.markers[id_] = marker;
    publisher_.publish(last_markers_);
  }

  //}

  /* cleanup() //{ */

  void StaticEdgesVisualization::cleanup()
  {
    last_markers_.markers[id_].action = visualization_msgs::msg::Marker::DELETE;
    is_active_ = false;
  }

  //}

}  // namespace mrs_lib
