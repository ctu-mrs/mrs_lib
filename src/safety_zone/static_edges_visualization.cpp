#include "mrs_lib/safety_zone/static_edges_visualization.h"
#include "mrs_lib/attitude_converter.h"
#include <geometry_msgs/Point.h>
#include <atomic>

namespace vm = visualization_msgs;
namespace gm = geometry_msgs;

namespace mrs_lib
{

  std::atomic<int> StaticEdgesVisualization::id_generator_ = 0;
  visualization_msgs::MarkerArray StaticEdgesVisualization::last_markers_ = visualization_msgs::MarkerArray();

  /* StaticEdgesVisualization() //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(Prism* prism, const std::string frame_id, const ros::NodeHandle nh, const double markers_update_rate)
      : id_(id_generator_), prism_(prism)
  {
    frame_id_ = frame_id;
    nh_ = nh;
    id_generator_++;
    timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &StaticEdgesVisualization::sendMarker, this);
    last_markers_.markers.push_back(vm::Marker());
    init();
  }

  //}

  /* StaticEdgesVisualization() //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(SafetyZone* safety_zone, const int obstacle_id, const std::string frame_id, const ros::NodeHandle nh,
                                                     double markers_update_rate)
      : id_(id_generator_), prism_(safety_zone->getObstacle(obstacle_id))
  {
    frame_id_ = frame_id;
    nh_ = nh;
    id_generator_++;
    timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &StaticEdgesVisualization::sendMarker, this);
    last_markers_.markers.push_back(vm::Marker());
    init();
  }

  //}

  /* StaticEdgesVisualization() //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(SafetyZone* safety_zone, const std::string frame_id, const ros::NodeHandle nh,
                                                     const double markers_update_rate)
      : id_(id_generator_), prism_(safety_zone->getBorder())
  {
    frame_id_ = frame_id;
    nh_ = nh;
    id_generator_++;
    last_markers_.markers.push_back(vm::Marker());
    timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &StaticEdgesVisualization::sendMarker, this);
    init();
  }

  //}

  /* StaticEdgesVisualization() //{ */

  StaticEdgesVisualization::~StaticEdgesVisualization()
  {
    if (is_active_)
    {
      prism_->unsubscribe(this);
    }
    cleanup();
  }

  //}

  /* init() //{ */

  void StaticEdgesVisualization::init()
  {
    publisher_ = nh_.advertise<vm::MarkerArray>("safety_area_static_markers_out", 1);
    prism_->subscribe(this);
    update();
    /* is_active_= true; */
  }

  //}

  /* sendMarker() //{ */

  // Lines must be updated periodically. View https://github.com/ros-visualization/rviz/issues/1287
  void StaticEdgesVisualization::sendMarker([[maybe_unused]] const ros::TimerEvent& event)
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

    const double max_z = prism_->getMaxZ();
    const double min_z = prism_->getMinZ();
    const auto polygon = prism_->getPolygon().outer();

    vm::Marker marker;
    marker.id = id_;
    marker.header.frame_id = frame_id_;
    marker.type = vm::Marker::LINE_LIST;
    marker.action = vm::Marker::ADD;
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
      gm::Point upper_start;
      gm::Point upper_end;

      upper_start.x = polygon[i].get<0>();
      upper_start.y = polygon[i].get<1>();
      upper_start.z = max_z;

      upper_end.x = polygon[i + 1].get<0>();
      upper_end.y = polygon[i + 1].get<1>();
      upper_end.z = max_z;

      marker.points.push_back(upper_start);
      marker.points.push_back(upper_end);

      // Adding lower horizontal edges
      gm::Point lower_start;
      gm::Point lower_end;

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
    last_markers_.markers[id_].action = vm::Marker::DELETE;
    is_active_ = false;
  }

  //}

}  // namespace mrs_lib
