#include "mrs_lib/safety_zone/yaml_export_visitor.h"
#include "mrs_lib/safety_zone/prism.h"
#include "mrs_lib/safety_zone/safety_zone.h"

#include <geometry_msgs/Point.h>

#include <sstream>

namespace gm = geometry_msgs;

namespace mrs_lib
{

  /* YamlExportVisitor() //{ */

  YamlExportVisitor::YamlExportVisitor(const std::string& prefix, const std::string& source_frame, const std::string& horizontal_frame,
                                       const std::string& vertical_frame, const std::string& units, const double origin_x, const double origin_y)
      : source_frame_(source_frame),
        horizontal_frame_(horizontal_frame),
        vertical_frame_(vertical_frame),
        transformer_(std::make_shared<mrs_lib::Transformer>("SafetyAreaManager"))
  {
    transformer_->setDefaultPrefix(prefix);

    std::stringstream ss;
    ss << "world_origin:" << std::endl
       << std::endl
       << std::setprecision(6) << std::fixed << "  units: \"" << units << "\"" << std::endl
       << std::endl
       << "  origin_x: " << origin_x << std::endl
       << "  origin_y: " << origin_y << std::endl
       << std::endl;

    world_origin_ = ss.str();

    ss.str("");
    ss << "safety_area:" << std::endl
       << std::endl
       << "  enabled: true" << std::endl
       << std::endl
       << "  horizontal_frame: \"" << horizontal_frame << "\"" << std::endl
       << "  vertical_frame: \"" << vertical_frame << "\"" << std::endl;

    safety_area_general_ = ss.str();

    ss.str("");
    ss << "    data: [" << std::endl;
    obstacle_points_ = ss.str();

    // obstacle_points_y_ = "      ";
    vertex_count_ = "    rows: [";
    obstacle_max_z_ = "    max_z: [";
    obstacle_min_z_ = "    min_z: [";
  }

  //}

  /* visit(safety_zone) //{ */

  void YamlExportVisitor::visit(SafetyZone* safety_zone)
  {
    std::stringstream ss;
    std::stringstream ss1;
    ss << "  border: " << std::endl << "    points: [" << std::endl << std::setprecision(6) << std::fixed;

    // ss1<< "      "
    //    << std::setprecision(6) << std::fixed;

    const auto polygon = safety_zone->getBorder()->getPolygon().outer();

    for (size_t i = 0; i < polygon.size() - 1; i++)
    {
      gm::Point point;
      point.x = polygon[i].get<0>();
      point.y = polygon[i].get<1>();
      point.z = 0;

      auto res = transformer_->transformSingle(source_frame_, point, horizontal_frame_);
      if (!res)
      {
        success_ = false;
        ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s during saving configuration", source_frame_.c_str(), horizontal_frame_.c_str());
        return;
      }

      point = res.value();
      ss << "      " << point.x << ", " << point.y << ",\n";
      // ss1 << point.y << ", ";
    }

    gm::Point point_max_z;
    gm::Point point_min_z;
    point_max_z.z = safety_zone->getBorder()->getMaxZ();
    point_min_z.z = safety_zone->getBorder()->getMinZ();

    auto res_max = transformer_->transformSingle(source_frame_, point_max_z, vertical_frame_);
    auto res_min = transformer_->transformSingle(source_frame_, point_min_z, vertical_frame_);

    if (!res_max || !res_min)
    {
      success_ = false;
      ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s during saving configuration", source_frame_.c_str(), vertical_frame_.c_str());
      return;
    }

    point_max_z = res_max.value();
    point_min_z = res_min.value();

    // ss << std::endl;
    ss << "    ]" << std::endl << "    max_z: " << point_max_z.z << std::endl << "    min_z: " << point_min_z.z << std::endl;

    // border_ = ss.str() + ss1.str();
    border_ = ss.str();
  }

  //}

  /* visit(obstacle) //{ */

  void YamlExportVisitor::visit(Prism* obstacle)
  {
    obstacles_present_ = true;

    std::stringstream ss;
    // std::stringstream ss1;

    // Writing points
    ss << std::setprecision(6) << std::fixed;
    // ss1 << std::setprecision(6) << std::fixed;

    const auto polygon = obstacle->getPolygon().outer();
    for (size_t i = 0; i < polygon.size() - 1; i++)
    {
      gm::Point point;
      point.x = polygon[i].get<0>();
      point.y = polygon[i].get<1>();
      point.z = 0;

      auto res = transformer_->transformSingle(source_frame_, point, horizontal_frame_);
      if (!res)
      {
        success_ = false;
        ROS_ERROR("[SafetyAreaManager]: Could not transform point from %s to %s during saving configuration", source_frame_.c_str(), horizontal_frame_.c_str());
        return;
      }

      point = res.value();
      ss << "      " << point.x << ", " << point.y << ",\n";
      // ss1 << point.y << ", ";
    }

    obstacle_points_ = obstacle_points_ + ss.str();
    // obstacle_points_y_ = obstacle_points_y_ + ss1.str();

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

    if (!res_max || !res_min)
    {
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

  //}

  /* getResult() //{ */

  std::string YamlExportVisitor::getResult()
  {
    if (!success_)
    {
      return "";
    }

    std::string present = std::string("  obstacles:\n    present: ") + (obstacles_present_ ? "true\n" : "false\n");

    obstacle_points_ = obstacle_points_ + "    ]\n";
    // obstacle_points_y_ = obstacle_points_y_ + "\n    ]\n";
    obstacle_max_z_ = obstacle_max_z_ + "]\n";
    obstacle_min_z_ = obstacle_min_z_ + "]\n";

    std::string cols = "    cols: 2\n";
    vertex_count_ = vertex_count_ + "]\n";

    std::string result = world_origin_ + safety_area_general_ + border_ + present;
    if (obstacles_present_)
    {
      result = result + obstacle_points_ + vertex_count_ + cols + obstacle_max_z_ + obstacle_min_z_;
    }

    return result;
  }

  //}

  /* isSuccessful() //{ */

  bool YamlExportVisitor::isSuccessful()
  {
    return success_;
  }

  //}

}  // namespace mrs_lib
