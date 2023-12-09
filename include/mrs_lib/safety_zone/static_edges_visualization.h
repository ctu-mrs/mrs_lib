#ifndef STATIC_EDGES_VISUALIZATION_H
#define STATIC_EDGES_VISUALIZATION_H

#include "prism.h"

#include <visualization_msgs/Marker.h>
#include <string>
#include <ros/ros.h>

namespace mrs_lib 
{
class StaticEdgesVisualization : public Subscriber{

public:
  StaticEdgesVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh, double markers_update_rate);

  void update();

private:
  void sendMarker(const ros::TimerEvent& event);

  Prism* prism_;
  std::string frame_id_;
  ros::NodeHandle nh_;

  // Static markers
  ros::Publisher publisher_;
  ros::Timer timer_;
  visualization_msgs::Marker last_marker_;

}; // class StaticEdgesVisualization
} // namespace mrs_lib


#endif