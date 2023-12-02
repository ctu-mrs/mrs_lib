#ifndef MRS_PRISM_VISUALIZATION_H
#define MRS_PRISM_VISUALIZATION_H

#include "prism.h"

#include "mrs_lib/publisher_handler.h"
#include <visualization_msgs/Marker.h>
#include <string>
#include <ros/ros.h>

namespace mrs_lib 
{
class PrismVisualization{

public:
  PrismVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh, double markers_update_rate);
  ~PrismVisualization() {}

private:
  void updateStaticMarkers(const ros::TimerEvent& event);

  Prism* prism_;
  std::string frame_id_;
  ros::NodeHandle nh_;

  // Static markers
  ros::Publisher static_markers_publisher_;
  ros::Timer static_markers_timer_;
  visualization_msgs::Marker static_marker_;
  bool static_markers_update_required_ = true;
}; // class PrismVisualization
} // namespace mrs_lib

#endif