#ifndef STATIC_EDGES_VISUALIZATION_H
#define STATIC_EDGES_VISUALIZATION_H

#include "prism.h"
#include "safety_zone.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <ros/ros.h>
#include <memory>
#include <atomic>

namespace mrs_lib
{
  class StaticEdgesVisualization final : public Subscriber
  {

  public:
    // Represents the prism
    StaticEdgesVisualization(Prism* prism, const std::string& frame_id, const ros::NodeHandle& nh, const double& markers_update_rate);

    // Represents corresponding obstacle in the safety_zone.
    StaticEdgesVisualization(SafetyZone* safety_zone, const int& obstacle_id, const std::string& frame_id, const ros::NodeHandle& nh,
                             const double& markers_update_rate);

    // Represents border of the safety_zone.
    StaticEdgesVisualization(SafetyZone* safety_zone, const std::string& frame_id, const ros::NodeHandle& nh, const double& markers_update_rate);

    ~StaticEdgesVisualization();

    void update() override;
    void cleanup() override;

  private:
    void init();
    void sendMarker([[maybe_unused]] const ros::TimerEvent& event);

    // It is required for generating static marker names,
    // because their id's must be unique on topic
    const int id_;
    static std::atomic<int> id_generator_;

    Prism* prism_;
    std::string frame_id_;
    ros::NodeHandle nh_;

    // Static markers
    ros::Publisher publisher_;
    ros::Timer timer_;
    static visualization_msgs::MarkerArray last_markers_;

  };  // class StaticEdgesVisualization
}  // namespace mrs_lib


#endif
