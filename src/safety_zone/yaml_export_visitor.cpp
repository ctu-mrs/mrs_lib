#include "mrs_lib/safety_zone/yaml_export_visitor.h"

#include <sstream> 

namespace mrs_lib {

YamlExportVisitor::YamlExportVisitor(std::string horizontal_frame, std::string vertical_frame,  std::string units, double origin_x, double origin_y) {
  transformer_ = std::make_shared<mrs_lib::Transformer>("SafetyAreaManager");
  
  horizontal_frame_ = horizontal_frame;
  vertical_frame_ = vertical_frame;
  // units_ = units;
  // origin_x_ = origin_x;
  // origin_y_ = origin_y;

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
}

void YamlExportVisitor::visit(SafetyZone* safety_zone) {
  std::stringstream ss;
  ss << "  boarder: " << std::endl
     << "    points: [" << std::endl
     << std::setprecision(6) << std::fixed;

  auto polygon = safety_zone->getBorder().getPolygon().outer();

  // TODO: add transformation
  for(int i=0; i<polygon.size(); i++) {
    ss << "      " << polygon[i].get<0>() << ", " << polygon[i].get<1>() << std::endl;
  }

  ss << "    ]" << std::endl
     << "    max_z: " << safety_zone->getBorder().getMaxZ() << std::endl
     << "    min_z: " << safety_zone->getBorder().getMinZ() << std::endl;

  border_ = ss.str();
}

void YamlExportVisitor::visit(Prism* prism, bool is_obstacle) {

}

std::string YamlExportVisitor::getResult() {
  return world_origin_ + safety_area_general_ + border_ + obstacles_;
}

} // namespace mrs_lib