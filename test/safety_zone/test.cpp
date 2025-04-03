#include <cmath>

#include <gtest/gtest.h>

#include <mrs_lib/safety_zone.h>

/* TEST(TESTSuite, point_valid_check) //{ */

TEST(TESTSuite, point_valid_check) {

  Eigen::MatrixXd border_points = Eigen::MatrixXd::Zero(4, 2);

  // clang-format off
  border_points << 0, 0,
                   10, 0,
                   10, 10,
                   0, 10;
  // clang-format on

  mrs_lib::safety_zone::SafetyZone safety_zone(border_points);

  EXPECT_TRUE(safety_zone.isPointValid(1, 1));
  EXPECT_TRUE(safety_zone.isPointValid(9, 9));

  EXPECT_FALSE(safety_zone.isPointValid(-10, -10));
  EXPECT_FALSE(safety_zone.isPointValid(100, 50));
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
