#ifndef EDGES_VISUALIZATION_H
#define EDGES_VISUALIZATION_H

#include "prism.h"

#include <visualization_msgs/Marker.h>
#include <string>
#include <ros/ros.h>

namespace mrs_lib 
{
class EdgesVisualization : public Subscriber{

public:
  EdgesVisualization(Prism* prism, std::string frame_id, ros::NodeHandle nh, double markers_update_rate);

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

}; // class EdgesVisualization
} // namespace mrs_lib


#endif