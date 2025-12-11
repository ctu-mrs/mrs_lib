#include "mrs_lib/safety_zone/static_edges_visualization.h"
#include <mrs_lib/attitude_converter.h>
#include <geometry_msgs/msg/point.h>
#include <atomic>

namespace mrs_lib
{

  std::atomic<int> StaticEdgesVisualization::id_generator_ = 0;

  /* StaticEdgesVisualization(prism) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(safety_zone::Prism prism, const std::string& uav_name, const std::string& frame_id,
                                                     const rclcpp::Node::SharedPtr node, const double& markers_update_rate)
      : id_(id_generator_++), prism_(prism), uav_name_(uav_name), frame_id_(frame_id), node_(node), last_markers_()
  {


    timer_ = node_->create_wall_timer(std::chrono::duration<double>(markers_update_rate), [this]() { sendMarker(); });
    init();
  }

  //}

  /* StaticEdgesVisualization(obstacle) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(safety_zone::SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name,
                                                     const std::string& frame_id, const rclcpp::Node::SharedPtr node, const double& markers_update_rate)

      : id_(id_generator_++), prism_(safety_zone->getObstacle(obstacle_id)), uav_name_(uav_name), frame_id_(frame_id), node_(node), last_markers_()

  {
    timer_ = node_->create_wall_timer(std::chrono::duration<double>(markers_update_rate), [this]() { sendMarker(); });
    init();
  }

  //}

  /* StaticEdgesVisualization(safety border) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(safety_zone::SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id,
                                                     const rclcpp::Node::SharedPtr node, const double& markers_update_rate)

      : id_(id_generator_++), prism_(safety_zone->getBorder()), uav_name_(uav_name), frame_id_(frame_id), node_(node), last_markers_()
  {

    timer_ = node_->create_wall_timer(std::chrono::duration<double>(markers_update_rate), [this]() { sendMarker(); });
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

    publisher_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(node_, "~/static_markers_out");
    coordinate_publisher_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(node_, "~/static_markers_coordinates_out");
    prism_.subscribe(this);
    update();
    is_active_ = true; // is_active_ flag inherited from Subscriber class defined in safety_zone::Prism
  }

  //}

  /* sendMarker() //{ */

  // Lines must be updated periodically. View https://github.com/ros-visualization/rviz/issues/1287
  void StaticEdgesVisualization::sendMarker()
  {
    publisher_.publish(last_markers_);
    coordinate_publisher_.publish(last_coordinates_);
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
    marker.ns = "static_edges_" + std::to_string(id_);
    std::string target_frame_id = "local_origin";
    marker.header.frame_id = uav_name_ + "/" + target_frame_id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    marker.scale.x = 0.2;

    visualization_msgs::msg::Marker safety_area_coordinates_marker;
    safety_area_coordinates_marker.ns = "coords_" + std::to_string(id_);
    ;
    std::string safety_area_coordinates_frame_id = "local_origin";
    safety_area_coordinates_marker.header.frame_id = uav_name_ + "/" + safety_area_coordinates_frame_id;
    safety_area_coordinates_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    safety_area_coordinates_marker.action = visualization_msgs::msg::Marker::ADD;
    safety_area_coordinates_marker.color.a = 1.0;
    safety_area_coordinates_marker.scale.z = 1.0;
    safety_area_coordinates_marker.color.r = 0.0;
    safety_area_coordinates_marker.color.g = 0.0;
    safety_area_coordinates_marker.color.b = 0.0;
    safety_area_coordinates_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    // Clear to prepare for new markers
    last_coordinates_.markers.clear();

    for (size_t i = 0; i < polygon.size() - 1; i++)
    { // -1 because the last is the same as the first one
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

      // Adding text markers for coordinates
      // Upper text marker
      std::stringstream ss;
      safety_area_coordinates_marker.id = i * 2 + 1;
      safety_area_coordinates_marker.pose.position.x = upper_start.x;
      safety_area_coordinates_marker.pose.position.y = upper_start.y;
      safety_area_coordinates_marker.pose.position.z = upper_start.z + 0.5;

      // Set white color for upper markers
      safety_area_coordinates_marker.color.r = 1.0;
      safety_area_coordinates_marker.color.g = 1.0;
      safety_area_coordinates_marker.color.b = 1.0;
      safety_area_coordinates_marker.color.a = 1.0;

      if (frame_id_ == "latlon_origin")
      {
        ss << std::fixed << std::setprecision(6) << "lat: " << std::to_string(upper_start.y).substr(0, 9)
           << "\n lon: " << std::to_string(upper_start.x).substr(0, 9) << "\n alt: " << std::to_string(upper_start.z).substr(0, 4);
      } else
      {
        ss << std::fixed << std::setprecision(2) << "x: " << std::to_string(upper_start.x).substr(0, 6)
           << " m\n y: " << std::to_string(upper_start.y).substr(0, 6) << " m\n z: " << std::to_string(upper_start.z).substr(0, 6) << " m";
      }

      safety_area_coordinates_marker.text = ss.str();

      last_coordinates_.markers.push_back(safety_area_coordinates_marker);

      // Clearing stringstream for reuse
      ss.str("");

      // Lower text marker
      safety_area_coordinates_marker.id = i * 2 + 2;
      safety_area_coordinates_marker.pose.position.x = lower_start.x;
      safety_area_coordinates_marker.pose.position.y = lower_start.y;
      safety_area_coordinates_marker.pose.position.z = lower_start.z - 0.5;

      // Set white color for lower markers
      safety_area_coordinates_marker.color.r = 0.0;
      safety_area_coordinates_marker.color.g = 0.0;
      safety_area_coordinates_marker.color.b = 0.0;
      safety_area_coordinates_marker.color.a = 1.0;

      if (frame_id_ == "latlon_origin")
      {
        ss.str("");
        ss << std::fixed << std::setprecision(6) << "lat: " << std::to_string(lower_start.y).substr(0, 9)
           << "\n lon: " << std::to_string(lower_start.x).substr(0, 9) << "\n alt: " << std::to_string(lower_start.z).substr(0, 4);
        safety_area_coordinates_marker.text = ss.str();
      } else
      {
        ss << std::fixed << std::setprecision(2) << "x: " << std::to_string(lower_start.x).substr(0, 6)
           << " m\n y: " << std::to_string(lower_start.y).substr(0, 6) << " m\n z: " << std::to_string(lower_start.z).substr(0, 6) << " m";
      }

      safety_area_coordinates_marker.text = ss.str();

      last_coordinates_.markers.push_back(safety_area_coordinates_marker);
    }

    if (last_markers_.markers.empty())
    {
      last_markers_.markers.push_back(visualization_msgs::msg::Marker());
    }

    last_markers_.markers[0] = marker;
    last_markers_.markers[0].id = id_; // Unique id for the marker
    publisher_.publish(last_markers_);
    coordinate_publisher_.publish(last_coordinates_);
  }

  //}

  /* cleanup() //{ */

  void StaticEdgesVisualization::cleanup()
  {
    if (!last_markers_.markers.empty())
    {
      last_markers_.markers[0].action = visualization_msgs::msg::Marker::DELETE;
      publisher_.publish(last_markers_);
    }

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.ns = "coords_" + std::to_string(id_);
    ;
    last_coordinates_.markers.clear();
    last_coordinates_.markers.push_back(delete_marker);
    coordinate_publisher_.publish(last_coordinates_);

    is_active_ = false;
  }

  //}

} // namespace mrs_lib
