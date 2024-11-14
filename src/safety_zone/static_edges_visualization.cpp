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

  /* StaticEdgesVisualization(prism) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(Prism* prism, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh, const double& markers_update_rate)
      : id_(id_generator_++), prism_(prism), uav_name_(uav_name), frame_id_(frame_id), nh_(nh)
  {
    timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &StaticEdgesVisualization::sendMarker, this);
    last_markers_.markers.push_back(vm::Marker());
    init();
  }

  //}

  /* StaticEdgesVisualization(obstacle) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(SafetyZone* safety_zone, const int& obstacle_id, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh,
                                                     const double& markers_update_rate)
      : id_(id_generator_++), prism_(safety_zone->getObstacle(obstacle_id)), uav_name_(uav_name), frame_id_(frame_id), nh_(nh)
  {
    timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &StaticEdgesVisualization::sendMarker, this);
    last_markers_.markers.push_back(vm::Marker());
    init();
  }

  //}

  /* StaticEdgesVisualization(safety border) //{ */

  StaticEdgesVisualization::StaticEdgesVisualization(SafetyZone* safety_zone, const std::string& uav_name, const std::string& frame_id, const ros::NodeHandle& nh,
                                                     const double& markers_update_rate, std::shared_ptr<mrs_lib::Transformer>& transformer)
      : id_(id_generator_++), prism_(safety_zone->getBorder()), uav_name_(uav_name), frame_id_(frame_id), nh_(nh), transformer_(transformer)
  {
    last_markers_.markers.push_back(vm::Marker());
    timer_ = nh_.createTimer(ros::Rate(markers_update_rate), &StaticEdgesVisualization::sendMarker, this);
    init();
  }

  //}

  /* ~StaticEdgesVisualization() //{ */

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
    is_active_ = true;  // is_active_ flag inherited from Subscriber class (defined in Prism)
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
    std::string target_frame_id = "world_origin";

    vm::Marker marker;
    marker.id = id_;
    marker.header.frame_id = uav_name_ +"/" + target_frame_id;
    marker.type = vm::Marker::LINE_LIST;
    marker.action = vm::Marker::ADD;
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    marker.scale.x = 0.2;

    /* old_version //{ */

    /* // Iterating over polygon edges to publish the marker lines */
    /* for (size_t i = 0; i < polygon.size() - 1; i++) */
    /* {  // -1 because the last is the same as the first one */
    /*   // Adding upper horizontal edges */
    /*   gm::Point upper_start; */
    /*   gm::Point upper_end; */

    /*   upper_start.x = polygon[i].get<0>(); */
    /*   upper_start.y = polygon[i].get<1>(); */
    /*   upper_start.z = max_z; */

    /*   upper_end.x = polygon[i + 1].get<0>(); */
    /*   upper_end.y = polygon[i + 1].get<1>(); */
    /*   upper_end.z = max_z; */

    /*   marker.points.push_back(upper_start); */
    /*   marker.points.push_back(upper_end); */

    /*   // Adding lower horizontal edges */
    /*   gm::Point lower_start; */
    /*   gm::Point lower_end; */

    /*   lower_start.x = polygon[i].get<0>(); */
    /*   lower_start.y = polygon[i].get<1>(); */
    /*   lower_start.z = min_z; */

    /*   lower_end.x = polygon[i + 1].get<0>(); */
    /*   lower_end.y = polygon[i + 1].get<1>(); */
    /*   lower_end.z = min_z; */

    /*   marker.points.push_back(lower_start); */
    /*   marker.points.push_back(lower_end); */

    /*   // Adding vertical edges */
    /*   marker.points.push_back(upper_start); */
    /*   marker.points.push_back(lower_start); */
    /* } */

    /* last_markers_.markers[id_] = marker; */
    /* publisher_.publish(last_markers_); */


    //}

    /* new_version //{ */
    // If we fail in transforming the area at some point
    // do not publish it at all
    bool tf_success = true;
    auto ret = transformer_->getTransform(frame_id_, target_frame_id , ros::Time(0));
    geometry_msgs::TransformStamped tf_viz;

    if (ret)
    {
      ROS_INFO_ONCE("[SafetyAreaManager]: got TFs, can publish safety area markers");
      tf_viz = ret.value();
    } else
    {
      tf_success = false;
    }


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

      auto [res1, transformed_upper_start] = transformPoint(upper_start, target_frame_id, tf_viz);
      marker.points.push_back(transformed_upper_start);

      auto [res2, transformed_upper_end] = transformPoint(upper_end, target_frame_id, tf_viz);
      marker.points.push_back(transformed_upper_end);

      // Adding lower horizontal edges
      gm::Point lower_start;
      gm::Point lower_end;

      lower_start.x = polygon[i].get<0>();
      lower_start.y = polygon[i].get<1>();
      lower_start.z = min_z;

      lower_end.x = polygon[i + 1].get<0>();
      lower_end.y = polygon[i + 1].get<1>();
      lower_end.z = min_z;

      auto [res3, transformed_lower_start] = transformPoint(lower_start, target_frame_id, tf_viz);
      marker.points.push_back(transformed_lower_start);

      auto [res4, transformed_lower_end] = transformPoint(lower_end, target_frame_id, tf_viz);
      marker.points.push_back(transformed_lower_end);

      if (!res1 || !res2 || !res3 || !res4)
      {
        tf_success = false;
      }

      // Adding vertical edges
      marker.points.push_back(transformed_upper_start);
      marker.points.push_back(transformed_lower_start);
    }

    if(tf_success)
    {
      last_markers_.markers[id_] = marker;
      publisher_.publish(last_markers_);

    }

    //}
  }

  //}

  /* transformPoint() //{ */

  std::tuple<bool, gm::Point> StaticEdgesVisualization::transformPoint(gm::Point point, std::string& frame_id, geometry_msgs::TransformStamped tf)
  {

    mrs_msgs::ReferenceStamped temp_ref;
    gm::Point transformed_point;
    temp_ref.header.frame_id = frame_id;
    temp_ref.header.stamp = ros::Time(0);
    temp_ref.reference.position.x = point.x;
    temp_ref.reference.position.y = point.y;
    temp_ref.reference.position.z = point.z;

    ROS_INFO_STREAM("[SafetyAreaManager]:  Original point x: " << point.x << " y: " << point.y << " z: " << point.z);

    if (auto ret = transformer_->transform(temp_ref, tf))
    {
      temp_ref = ret.value();
      ROS_INFO_STREAM("[SafetyAreaManager]: Transformed point x: " << temp_ref.reference.position.x << " y: " << temp_ref.reference.position.y
                                                                               << " z: " << temp_ref.reference.position.z);
    } else
    {
      return std::make_tuple(false, transformed_point);
    }

    transformed_point.x = temp_ref.reference.position.x;
    transformed_point.y = temp_ref.reference.position.y;
    transformed_point.z = temp_ref.reference.position.z;

    return std::make_tuple(true, transformed_point);
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
