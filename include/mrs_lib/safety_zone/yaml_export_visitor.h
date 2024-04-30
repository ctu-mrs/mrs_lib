#ifndef YAML_EXPORT_VISITOR_H
#define YAML_EXPORT_VISITOR_H

#include "mrs_lib/transformer.h"

namespace mrs_lib
{

class Prism;
class SafetyZone;

class Visitor{
public:
  virtual void visit(SafetyZone* safety_zone) = 0;

  virtual void visit(Prism* obstacle) = 0;
};

class YamlExportVisitor : public Visitor{
private:
  std::string source_frame_;
  std::string horizontal_frame_;
  std::string vertical_frame_;
  std::string vertex_count_;
  bool success_ = true;
  bool obstacles_present_ = false;

  // Texts
  std::string world_origin_;
  std::string safety_area_general_;
  std::string border_;
  std::string obstacle_points_;
  std::string obstacle_max_z_;
  std::string obstacle_min_z_;

  std::shared_ptr<Transformer> transformer_;

public:
  // Does not transform origin_x and origin_y to units!
  YamlExportVisitor(std::string prefix, std::string source_frame, std::string horizontal_frame, std::string vertical_frame,  std::string units, double origin_x, double origin_y);

  // Only takes a border into consideration
  void visit(SafetyZone* safety_zone);

  void visit(Prism* obstacle);

  std::string getResult();

  bool isSuccessful();
};
} // namespace mrs_lib

#endif