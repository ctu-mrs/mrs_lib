#ifndef YAML_EXPORT_VISITOR_H
#define YAML_EXPORT_VISITOR_H

#include "prism.h"
#include "safety_zone.h"
#include "mrs_lib/transformer.h"


namespace mrs_lib
{
class Visitor{
  virtual void visit(SafetyZone* safety_zone) = 0;

  virtual void visit(Prism* prism, bool is_obstacle) = 0;
};

class YamlExportVisitor : public Visitor{
private:
  std::string horizontal_frame_;
  std::string vertical_frame_;
  // std::string units_;
  // double origin_x_;
  // double origin_y_;

  // Texts
  std::string world_origin_;
  std::string safety_area_general_;
  std::string border_;
  std::string obstacles_;

  std::shared_ptr<Transformer> transformer_;

public:
  // Does not transform origin_x and origin_y to units!
  YamlExportVisitor(std::string horizontal_frame, std::string vertical_frame,  std::string units, double origin_x, double origin_y);

  void visit(SafetyZone* safety_zone);
  void visit(Prism* prism, bool is_obstacle);

  std::string getResult();
};
} // namespace mrs_lib

#endif