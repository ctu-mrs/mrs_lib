#include <mrs_lib/SafetyZone/SafetyZone.h>
#include <mrs_lib/SafetyZone/lineOperations.h>

namespace mrs_lib
{
  /* SafetyZone() //{ */

  SafetyZone::SafetyZone(mrs_lib::Polygon border, std::vector<Polygon> innerObstacles, std::vector<PointObstacle> pointObstacles)
      : innerObstacles(innerObstacles), pointObstacles(pointObstacles)
  {
    outerBorder = new Polygon(border);
  }

  //}

  /* SafetyZone() //{ */

  SafetyZone::~SafetyZone()
  {
    delete outerBorder;
  }

  //}

  /* SafetyZone() //{ */

  SafetyZone::SafetyZone(const Eigen::MatrixXd& outerBorderMatrix, const std::vector<Eigen::MatrixXd>& innerObstaclesMatrixes,
                         const std::vector<Eigen::MatrixXd>& pointObstaclesMatrixes)
  {
    try
    {
      outerBorder = new Polygon(outerBorderMatrix);
    }
    catch (Polygon::WrongNumberOfVertices)
    {
      throw BorderError();
    }
    catch (Polygon::WrongNumberOfColumns)
    {
      throw BorderError();
    }
    catch (Polygon::ExtraVertices)
    {
      throw BorderError();
    }

    for (auto& matrix : innerObstaclesMatrixes)
    {
      try
      {
        innerObstacles.emplace_back(matrix);
      }
      catch (Polygon::WrongNumberOfVertices)
      {
        throw PolygonObstacleError();
      }
      catch (Polygon::WrongNumberOfColumns)
      {
        throw PolygonObstacleError();
      }
      catch (Polygon::ExtraVertices)
      {
        throw PolygonObstacleError();
      }
    }

    for (auto& matrix : pointObstaclesMatrixes)
    {
      if (matrix.cols() != 4 || matrix.rows() != 1)
      {
        throw PointObstacleError();
      }

      pointObstacles.emplace_back(Eigen::RowVector2d{matrix(0, 0), matrix(0, 1)}, matrix(0, 2), matrix(0, 3));
    }
  }

  //}

  /* isPointValid() //{ */

  bool SafetyZone::isPointValid(const double px, const double py, const double pz)
  {

    if (!outerBorder->isPointInside(px, py))
    {
      return false;
    }

    for (auto& elem : innerObstacles)
    {
      if (elem.isPointInside(px, py))
      {
        return false;
      }
    }

    for (auto& elem : pointObstacles)
    {
      if (elem.isPointInside(px, py, pz))
      {
        return false;
      }
    }

    return true;
  }

  //}

  /* isPathValid() //{ */

  bool SafetyZone::isPathValid(const double p1x, const double p1y, const double p1z, const double p2x, const double p2y, const double p2z)
  {

    if (outerBorder->doesSectionIntersect(p1x, p1y, p2x, p2y))
      return false;

    for (auto& el : innerObstacles)
    {
      if (el.doesSectionIntersect(p1x, p1y, p1z, p2x))
        return false;
    }

    for (auto& el : pointObstacles)
    {
      if (el.doesSectionIntersect(p1x, p1y, p1z, p2x, p2y, p2z))
        return false;
    }

    return true;
  }

  //}

  /* getBorder() //{ */

  void SafetyZone::inflateBorder(const double amount)
  {
    outerBorder->inflateSelf(amount);
  }

  //}

  /* getBorder() //{ */

  Polygon SafetyZone::getBorder()
  {
    return *outerBorder;
  }

  //}

  /* getObstacles() //{ */

  std::vector<Polygon> SafetyZone::getObstacles()
  {
    return innerObstacles;
  }

  //}

  /* getPointObstacles() //{ */

  std::vector<PointObstacle> SafetyZone::getPointObstacles()
  {
    return pointObstacles;
  }

  //}

  /*  //{ */

  visualization_msgs::Marker SafetyZone::toMarker(const double min_height, const double max_height)
  {
    visualization_msgs::Marker safety_area_marker;
  
    std::cerr << "SAFETY min " << min_height << "max " << max_height << std::endl;

    safety_area_marker.type            = visualization_msgs::Marker::LINE_LIST;
    safety_area_marker.color.a         = 0.15;
    safety_area_marker.scale.x         = 0.2;
    safety_area_marker.color.r         = 0;
    safety_area_marker.color.g         = 0;
    safety_area_marker.color.b         = 1;
  
    safety_area_marker.pose.orientation.x = 0;
    safety_area_marker.pose.orientation.y = 0;
    safety_area_marker.pose.orientation.z = 0;
    safety_area_marker.pose.orientation.w = 1;

  
    /* adding safety area points //{ */
  
    std::vector<geometry_msgs::Point> border_points_bot = getBorder().getPointMessageVector(min_height);
    // bottom border
    for (size_t i = 0; i < border_points_bot.size(); i++)
    {
      safety_area_marker.points.push_back(border_points_bot.at(i));
      safety_area_marker.points.push_back(border_points_bot.at((i + 1) % border_points_bot.size()));
    }

    std::vector<geometry_msgs::Point> border_points_top = getBorder().getPointMessageVector(max_height);
    // top border
    for (size_t i = 0; i < border_points_top.size(); i++)
    {
      safety_area_marker.points.push_back(border_points_top.at(i));
      safety_area_marker.points.push_back(border_points_top.at((i + 1) % border_points_top.size()));
    }

    // top/bot edges
    for (size_t i = 0; i < border_points_top.size(); i++)
    {
      safety_area_marker.points.push_back(border_points_top.at(i));
      safety_area_marker.points.push_back(border_points_bot.at(i));
    }
  
    //}
  
    /* adding polygon obstacles points //{ */
  
    std::vector<mrs_lib::Polygon> polygon_obstacles = getObstacles();
  
    for (auto polygon : polygon_obstacles) {
  
      std::vector<geometry_msgs::Point> points_bot = polygon.getPointMessageVector(min_height);
      std::vector<geometry_msgs::Point> points_top = polygon.getPointMessageVector(max_height);
  
      // bottom points
      for (size_t i = 0; i < points_bot.size(); i++) {
  
        safety_area_marker.points.push_back(points_bot[i]);
        safety_area_marker.points.push_back(points_bot[(i + 1) % points_bot.size()]);
      }
  
      // top points + top/bot edges
      for (size_t i = 0; i < points_bot.size(); i++) {
  
        safety_area_marker.points.push_back(points_top[i]);
        safety_area_marker.points.push_back(points_top[(i + 1) % points_top.size()]);
  
        safety_area_marker.points.push_back(points_bot[i]);
        safety_area_marker.points.push_back(points_top[i]);
      }
    }
  
    //}
  
    /* adding point-obstacle points //{ */
  
    std::vector<mrs_lib::PointObstacle> point_obstacles = getPointObstacles();
  
    for (auto point : point_obstacles) {
  
      std::vector<geometry_msgs::Point> points_bot = point.getPointMessageVector(min_height);
      std::vector<geometry_msgs::Point> points_top = point.getPointMessageVector(-1);
  
      // botom points
      for (size_t i = 0; i < points_bot.size(); i++) {
  
        safety_area_marker.points.push_back(points_bot[i]);
        safety_area_marker.points.push_back(points_bot[(i + 1) % points_bot.size()]);
      }
  
      // top points + bot/top edges
      for (size_t i = 0; i < points_top.size(); i++) {
  
        safety_area_marker.points.push_back(points_top[i]);
        safety_area_marker.points.push_back(points_top[(i + 1) % points_top.size()]);
  
        safety_area_marker.points.push_back(points_bot[i]);
        safety_area_marker.points.push_back(points_top[i]);
      }
    }
  
    //}
  
    return safety_area_marker;
  }
  
  //}

}  // namespace mrs_lib
