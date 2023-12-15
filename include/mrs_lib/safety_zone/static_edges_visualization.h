#ifndef STATIC_EDGES_VISUALIZATION_H
#define STATIC_EDGES_VISUALIZATION_H

#include "prism.h"
#include "safety_zone.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <ros/ros.h>

namespace mrs_lib 
{
class StaticEdgesVisualization : public Subscriber{

public:
  StaticEdgesVisualization(Prism& prism, std::string frame_id, ros::NodeHandle nh, double markers_update_rate);
  StaticEdgesVisualization(SafetyZone* safety_zone, int obstacle_id, std::string frame_id, ros::NodeHandle nh, double markers_update_rate);
  StaticEdgesVisualization(SafetyZone* safety_zone, std::string frame_id, ros::NodeHandle nh, double markers_update_rate);

  ~StaticEdgesVisualization();

  void update();
  void cleanup();

private:
  void init();
  void sendMarker(const ros::TimerEvent& event);

  const int id_;
  static int id_generator_;

  Prism& prism_;
  std::string frame_id_;
  ros::NodeHandle nh_;

  // Static markers
  ros::Publisher publisher_;
  ros::Timer timer_;
  static visualization_msgs::MarkerArray last_markers_;

}; // class StaticEdgesVisualization
} // namespace mrs_lib


#endif