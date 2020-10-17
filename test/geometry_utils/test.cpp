#include <mrs_lib/geometry_utils.h>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

#include <random>

using namespace mrs_lib;
using namespace std;

/* randd() //{ */

double randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* unwrapAngleTest() //{ */

double unwrapAngleTest(const double& yaw, const double& yaw_previous) {

  double yaw_out = yaw;

  while (yaw_out - yaw_previous > M_PI) {
    yaw_out -= 2 * M_PI;
  }

  while (yaw_out - yaw_previous < -M_PI) {
    yaw_out += 2 * M_PI;
  }

  return yaw_out;
}

//}

/* wrapAngleTest() //{ */

double wrapAngleTest(const double& angle_in) {

  double angle_wrapped = angle_in;

  while (angle_wrapped > M_PI) {
    angle_wrapped -= 2 * M_PI;
  }

  while (angle_wrapped < -M_PI) {
    angle_wrapped += 2 * M_PI;
  }

  return angle_wrapped;
}

//}

/* TEST(TESTSuite, rotation_between_1) //{ */

TEST(TESTSuite, rotation_between_1) {

  int result = 1;

  const Eigen::Vector3d vec1(1, 0, 0);
  const Eigen::Vector3d vec2(0, 0, -10);

  const auto hvec1 = to_homogenous(vec1);

  cout << hvec1 << std::endl;

  const auto rot_iden = rotation_between(vec1, vec1);

  double angle = angle_between(vec1, vec1);

  cout << "Should be identity (angle: " << angle << "):" << std::endl << rot_iden << std::endl;

  if (fabs(angle) >= 1e-6) {
    result = 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotation_between_2) //{ */

TEST(TESTSuite, rotation_between_2) {

  int result = 0;

  const Eigen::Vector3d vec1(1, 0, 0);
  const Eigen::Vector3d vec2(0, 0, -10);

  const auto rot_90 = rotation_between(vec1, vec2);

  double angle = angle_between(vec1, vec2);

  cout << "Should be 90 deg (angle: " << angle << "):" << std::endl << rot_90 << std::endl;

  if (fabs(angle - (M_PI / 2.0)) <= 1e-6 || (fabs(angle + (M_PI / 2))) <= 1e-6) {
    result = 1;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotation_between_3) //{ */

TEST(TESTSuite, rotation_between_3) {

  int result = 1;

  const Eigen::Vector3d vec1(1, 0, 0);
  const Eigen::Vector3d vec3(1, 0, 1);

  const auto hvec1 = to_homogenous(vec1);

  cout << hvec1 << std::endl;

  const auto rot_45 = rotation_between(vec1, vec3);

  double angle = angle_between(vec1, vec3);

  cout << "Should be 45 deg (angle: " << angle << "):" << std::endl << rot_45 << std::endl;

  if (fabs(angle - (M_PI / 4.0)) >= 1e-6) {
    result = 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotation_between_4) //{ */

TEST(TESTSuite, rotation_between_4) {

  int result = 1;

  const Eigen::Vector2d vec4(-5, 0);

  double angle = angle_between(vec4, vec4);

  cout << "angle from vec4 to vec4 (should be 0): " << angle << std::endl;

  if (fabs(angle) >= 1e-6) {
    result = 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotation_between_5) //{ */

TEST(TESTSuite, rotation_between_5) {

  int result = 0;

  const Eigen::Vector2d vec4(-5, 0);
  const Eigen::Vector2d vec5(10, 0);

  double angle = angle_between(vec4, vec5);

  cout << "angle from vec4 to vec5 (should be +-pi): " << angle << std::endl;

  if (fabs(angle - M_PI) >= 1e-6 || fabs(angle + M_PI) >= 1e-6) {
    result = 1;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotation_between_6) //{ */

TEST(TESTSuite, rotation_between_6) {

  int result = 0;

  const Eigen::Vector2d vec4(-5, 0);
  const Eigen::Vector2d vec6(-1, 1);

  double angle = angle_between(vec4, vec6);

  cout << "angle from vec4 to vec6 (should be -pi/4): " << angle << std::endl;

  if (fabs(angle + (M_PI / 4.0)) <= 1e-6) {
    result = 1;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, unwrap_angle) //{ */

TEST(TESTSuite, unwrap_angle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double previous_angle = randd(-10000, 10000);
    double current_angle  = randd(-10000, 10000);

    double output   = mrs_lib::unwrapAngle(current_angle, previous_angle);
    double expected = unwrapAngleTest(current_angle, previous_angle);

    if (fabs(output - expected) > 1e-6) {
      printf("unwrapAngle() #%d faild, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, wrap_angle) //{ */

TEST(TESTSuite, wrap_angle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double angle = randd(-10000, 10000);

    double output   = mrs_lib::wrapAngle(angle);
    double expected = wrapAngleTest(angle);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  // initialize the random number generator
  /* srand(static_cast<unsigned int>(time(0))); */
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
