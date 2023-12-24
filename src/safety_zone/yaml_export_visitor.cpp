#include "mrs_lib/safety_zone/yaml_export_visitor.h"
#include "mrs_lib/safety_zone/prism.h"
#include "mrs_lib/safety_zone/safety_zone.h"

#include <geometry_msgs/Point.h>

#include <sstream> 

namespace gm = geometry_msgs;

namespace mrs_lib {

YamlExportVisitor::YamlExportVisitor(std::string prefix, std::string source_frame, std::string horizontal_frame, std::string vertical_frame,  std::string units, double origin_x, double origin_y) {
  transformer_ = std::make_shared<mrs_lib::Transformer>("SafetyAreaManager");
  transformer_->setDefaultPrefix(prefix);
  
  source_frame_ = source_frame;
  horizontal_frame_ = horizontal_frame;
  vertical_frame_ = vertical_frame;
  
  std::stringstream ss;
  ss << "world_origin:" << std::endl << std::endl
     << std::setprecision(6) << std::fixed
     << "  units: \"" << units << "\"" << std::endl << std::endl 
     << "  origin_x: " << origin_x << std::endl
     << "  origin_y: " << origin_y << std::endl << std::endl;

  world_origin_ = ss.str();

  ss.str("");
  ss << "safety_area:" << std::endl << std::endl 
     << "  enabled: true" << std::endl << std::endl
     << "  horizontal_frame: \"" << horizontal_frame << "\"" << std::endl
     << "  vertical_frame: \"" << vertical_frame << "\"" << std::endl; 

  safety_area_general_ = ss.str();

  ss.str("");
  ss << "  obstacles:" << std::endl
     << "    points: [" << std::endl;
  obstacle_points_ = ss.str();

  vertex_count_ = "    rows: [";
  obstacle_max_z_ = "    max_z: [";
  obstacle_min_z_ = "    min_z: [";
}

void YamlExportVisitor::visit(SafetyZone* safety_zone) {
  std::stringstream ss;
  ss << "  boarder: " << std::endl
     << "    points: [" << std::endl
     << std::setprecision(6) << std::fixed;

  auto polygon = safety_zone->getBorder().getPolygon().outer();

  // TODO: add transformation
  for(int i=0; i<polygon.size(); i++) {
    gm::Point point;
    point.x = polygon[i].get<0>();
    point.y = polygon[i].get<1>();
    point.z = 0;

    auto res = transformer_->transformSingle(source_frame_, point, horizontal_frame_);
    if(!res){
      success_ = false;
      ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s during saving configuration", source_frame_.c_str(), horizontal_frame_.c_str());
      return;
    }

    point = res.value();
    ss << "      " << point.x << ", " << point.y << std::endl;
  }

  gm::Point point_max_z;
  gm::Point point_min_z;
  point_max_z.z = safety_zone->getBorder().getMaxZ();
  point_min_z.z = safety_zone->getBorder().getMinZ();

  auto res_max = transformer_->transformSingle(source_frame_, point_max_z, vertical_frame_);
  auto res_min = transformer_->transformSingle(source_frame_, point_min_z, vertical_frame_);

  if(!res_max || !res_min) {
      success_ = false;
      ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s during saving configuration", source_frame_.c_str(), vertical_frame_.c_str());
      return;
  }

  point_max_z = res_max.value();
  point_min_z = res_min.value();

  ss << "    ]" << std::endl
     << "    max_z: " << point_max_z.z << std::endl
     << "    min_z: " << point_min_z.z << std::endl;

  border_ = ss.str();
}

void YamlExportVisitor::visit(Prism* obstacle) {
  std::stringstream ss;

  // Writing points
  ss //<< "    [" << std::endl
     << std::setprecision(6) << std::fixed;
  
  auto polygon = obstacle->getPolygon().outer();
  for(int i=0; i<polygon.size() - 1; i++) {gm::Point point;
    point.x = polygon[i].get<0>();
    point.y = polygon[i].get<1>();
    point.z = 0;

    auto res = transformer_->transformSingle(source_frame_, point, horizontal_frame_);
    if(!res){
      success_ = false;
      ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s during saving configuration", source_frame_.c_str(), horizontal_frame_.c_str());
      return;
    }

    point = res.value();
    ss << "      " << point.x << ", " << point.y << std::endl;
  }

  // ss << "    ]," << std::endl;

  obstacle_points_ = obstacle_points_ + ss.str();

  // Writing number of rows in matrix (number of vertices)
  ss.str("");
  ss << polygon.size() - 1 << ", ";
  vertex_count_ = vertex_count_ + ss.str();

  // Writing max and min z
  gm::Point point_max_z;
  gm::Point point_min_z;
  point_max_z.z = obstacle->getMaxZ();
  point_min_z.z = obstacle->getMinZ();

  auto res_max = transformer_->transformSingle(source_frame_, point_max_z, vertical_frame_);
  auto res_min = transformer_->transformSingle(source_frame_, point_min_z, vertical_frame_);

  if(!res_max || !res_min) {
      success_ = false;
      ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s during saving configuration", source_frame_.c_str(), vertical_frame_.c_str());
      return;
  }

  point_max_z = res_max.value();
  point_min_z = res_min.value();

  ss.str("");
  ss << point_max_z.z << ", ";
  obstacle_max_z_ = obstacle_max_z_ + ss.str();

  ss.str("");
  ss << point_min_z.z << ", ";
  obstacle_min_z_ = obstacle_min_z_ + ss.str();
}

std::string YamlExportVisitor::getResult() {
  if(!success_){
    return "";
  }

  obstacle_points_ = obstacle_points_ + "    ]\n";
  obstacle_max_z_ = obstacle_max_z_ + "]\n";
  obstacle_min_z_ = obstacle_min_z_ + "]\n";

  std::string cols = "    cols: 2\n";
  vertex_count_ = vertex_count_ + "]\n";

  return world_origin_ + safety_area_general_ + border_ + obstacle_points_ + vertex_count_ + cols + obstacle_max_z_ + obstacle_min_z_ ;
}

bool YamlExportVisitor::isSuccessful(){
  return success_;
}

} // namespace mrs_lib