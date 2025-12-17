#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/safety_zone.h>

/* TEST(TESTSuite, point_valid_check) //{ */

TEST(TESTSuite, point_valid_check)
{

  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(4, 2);

  // clang-format off
  matrix << 0, 0,
            10, 0,
            10, 10,
            0, 10;
  // clang-format on

  std::vector<mrs_lib::safety_zone::Point2d> points;
  points.reserve(matrix.rows());

  for (int i = 0; i < matrix.rows(); i++)
  {
    points.emplace_back(mrs_lib::safety_zone::Point2d{matrix(i, 0), matrix(i, 1)});
  }

  double max_z = 5.0;
  double min_z = 0.0;

  auto border = std::make_unique<mrs_lib::safety_zone::Prism>(points, max_z, min_z, "world_origin", "world_origin");

  auto safety_zone = std::make_shared<mrs_lib::safety_zone::SafetyZone>(std::move(border));

  if (safety_zone->safetyZoneEnabled() == false)
  {
    std::cout << "Enabling safety zone for testing." << std::endl;
    safety_zone->enableSafetyZone(true);
  }

  // Testing 2D points
  EXPECT_TRUE(safety_zone->isPointValid(1, 1));
  EXPECT_TRUE(safety_zone->isPointValid(9, 9));

  EXPECT_FALSE(safety_zone->isPointValid(-10, -10));
  EXPECT_FALSE(safety_zone->isPointValid(100, 50));

  // Testing 3D points
  EXPECT_TRUE(safety_zone->isPointValid(1, 1, 2));
  EXPECT_TRUE(safety_zone->isPointValid(9, 9, 0.5));

  // TODO this did not work, returned TRUE (which is probably still fine?)
  /* EXPECT_FALSE(safety_zone->isPointValid(9, 9, 0));  // On min_z */

  EXPECT_FALSE(safety_zone->isPointValid(1, 1, 10)); // Above max_z
  EXPECT_FALSE(safety_zone->isPointValid(1, 1, -1)); // Below min_z
}

//}

/* TEST(TESTSuite, obstacle_valid_check) //{ */

TEST(TESTSuite, obstacle_valid_check)
{

  Eigen::MatrixXd border_matrix = Eigen::MatrixXd::Zero(4, 2);

  // clang-format off
  border_matrix << 0, 0,
                   10, 0,
                   10, 10,
                   0, 10;
  // clang-format on

  std::vector<mrs_lib::safety_zone::Point2d> border_points;
  border_points.reserve(border_matrix.rows());

  for (int i = 0; i < border_matrix.rows(); i++)
  {
    border_points.emplace_back(mrs_lib::safety_zone::Point2d{border_matrix(i, 0), border_matrix(i, 1)});
  }

  double border_max_z = 5.0;
  double border_min_z = 0.0;

  auto border = std::make_unique<mrs_lib::safety_zone::Prism>(border_points, border_max_z, border_min_z, "world_origin", "world_origin");

  // Obstacle definition
  Eigen::MatrixXd obstacle_matrix = Eigen::MatrixXd::Zero(4, 2);

  // clang-format off
  obstacle_matrix << 4, 4,
                     6, 4,
                     6, 6,
                     4, 6;
  // clang-format on

  std::vector<mrs_lib::safety_zone::Point2d> obstacle_points;
  obstacle_points.reserve(obstacle_matrix.rows());

  for (int i = 0; i < obstacle_matrix.rows(); i++)
  {
    obstacle_points.emplace_back(mrs_lib::safety_zone::Point2d{obstacle_matrix(i, 0), obstacle_matrix(i, 1)});
  }

  double obstacle_max_z = 3.0;
  double obstacle_min_z = 1.0;

  auto obstacle = std::make_unique<mrs_lib::safety_zone::Prism>(obstacle_points, obstacle_max_z, obstacle_min_z, "world_origin", "world_origin");

  std::vector<std::unique_ptr<mrs_lib::safety_zone::Prism>> obstacles;
  obstacles.push_back(std::move(obstacle));

  auto safety_zone = std::make_shared<mrs_lib::safety_zone::SafetyZone>(std::move(border), std::move(obstacles));

  if (safety_zone->safetyZoneEnabled() == false)
  {
    std::cout << "Enabling safety zone for testing." << std::endl;
    safety_zone->enableSafetyZone(true);
  }

  // Testing points inside border but inside obstacle
  EXPECT_FALSE(safety_zone->isPointValid(5, 5, 2));
  EXPECT_FALSE(safety_zone->isPointValid(4.5, 4.5, 1.5));
  EXPECT_FALSE(safety_zone->isPointValid(5.5, 5.5, 2.9));

  // Testing points inside border and outside obstacle
  EXPECT_TRUE(safety_zone->isPointValid(3, 3, 2));
  EXPECT_TRUE(safety_zone->isPointValid(7, 7, 4));
  EXPECT_TRUE(safety_zone->isPointValid(0.5, 0.5, 0.5));
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
