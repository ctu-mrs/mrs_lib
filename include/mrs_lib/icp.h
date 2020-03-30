#include <Eigen/Dense>
#include <ros/ros.h>

namespace e = Eigen;
namespace icp {
  class ICPSolver {
    public:
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> copyShape(std::vector<std::pair<e::Vector2d,e::Vector2d>> shape);
      static e::Vector2d flatAverage(std::vector<e::Vector2d> input);
      static e::Vector3d fullAverage(std::vector<e::Vector3d> input);
      static double totalError(std::vector<std::pair<e::Vector2d,e::Vector2d>> ref_shape, std::vector<e::Vector2d> input);
      static double optimizeRotation(std::vector<e::Vector2d> input, std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d center);
      static double optimizeFull(std::vector<e::Vector2d> input, std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d &center, e::Vector3d &gradient);
      static e::Vector3d matchShape(std::vector<e::Vector3d> input, std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector3d &gradient);
      static e::Vector3d matchSquare(std::vector<e::Vector3d> input, double side, e::Vector3d &gradient);
      static e::Vector3d matchTriangle(std::vector<e::Vector3d> input, double side, e::Vector3d &gradient);
      static e::Vector3d matchLine(std::vector<e::Vector3d> input, double side, e::Vector3d &gradient);
    private:
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> translateShape(std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d pos);
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> rotateShape(std::vector<std::pair<e::Vector2d,e::Vector2d>> shape, e::Vector2d center, double angle);
      static double distSquarePointSegment(e::Vector2d point, std::pair<e::Vector2d,e::Vector2d> segment);
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> initializeYaw(std::vector<e::Vector2d> input, std::vector<std::pair<e::Vector2d,e::Vector2d>> ref_shape, double yaw_step, e::Vector2d center, double& yaw_best);
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> makeSquare(double side);
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> makeSquareChamfered(double side);
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> makeTriangle(double side);
      static std::vector<std::pair<e::Vector2d,e::Vector2d>> makeLine(double side);
  };
}
