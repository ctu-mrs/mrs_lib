#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <cmath>
#include <iostream>
#include <complex>

#include <gtest/gtest.h>

using namespace mrs_lib::geometry;
using namespace std;
using cx = std::complex<double>;

template <typename T>
int approxEqual(const T& what, const double& to, const double& tol = 1e-9)
{
  return std::abs(what - to) < tol;
}

/* randd() //{ */

double randd(double from, double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* addAngleTest() //{ */

double addAngleTest(const double a, const double b) {
  return std::arg(std::polar<double>(1, a + b));
}

//}

/* diffAngleTest() //{ */

double diffAngleTest(const double a, const double b) {
  return std::arg(std::polar<double>(1, a) * std::conj(std::polar<double>(1, b)));
}

//}

/* distAngleTest() //{ */

static double distAngleTest(const double a, const double b) {
  return std::abs(diffAngleTest(a, b));
}

//}

/* wrapAngleTest() //{ */

double wrapAngleTest(const double angle_in, const double angle_min = -M_PI, const double angle_max = M_PI) {

  const double angle_range   = angle_max - angle_min;
  double       angle_wrapped = angle_in;

  while (angle_wrapped > angle_max) {
    angle_wrapped -= angle_range;
  }

  while (angle_wrapped < angle_min) {
    angle_wrapped += angle_range;
  }

  return angle_wrapped;
}

//}

/* unwrapAngleTest() //{ */

double unwrapAngleTest(const double ang, const double ang_previous, const double angle_range = 2 * M_PI) {
  const double nega     = -angle_range / 2.0;
  const double posa     = angle_range / 2.0;
  const double ang_diff = wrapAngleTest(ang - ang_previous, nega, posa);
  return ang_previous + ang_diff;
}

//}

/* interpolateAngles() //{ */

double interpolateAngleTest(const double a1, const double a2, const double coeff) {
  Eigen::Vector3d axis = Eigen::Vector3d(0, 0, 1);

  Eigen::Quaterniond quat1 = Eigen::Quaterniond(Eigen::AngleAxis<double>(a1, axis));
  Eigen::Quaterniond quat2 = Eigen::Quaterniond(Eigen::AngleAxis<double>(a2, axis));

  Eigen::Quaterniond new_quat = quat1.slerp(coeff, quat2);

  Eigen::Vector3d vecx = new_quat * Eigen::Vector3d(1, 0, 0);

  return atan2(vecx[1], vecx[0]);
}

//}

/* TEST(TESTSuite, rotationBetween1) //{ */

TEST(TESTSuite, rotationBetween1) {

  int result = 1;

  const Eigen::Vector3d vec1(1, 0, 0);
  const Eigen::Vector3d vec2(0, 0, -10);

  const auto hvec1 = toHomogenous(vec1);

  cout << hvec1 << std::endl;

  const auto rot_iden = rotationBetween(vec1, vec1);

  double angle = angleBetween(vec1, vec1);

  cout << "Should be identity (angle: " << angle << "):" << std::endl << rot_iden << std::endl;

  if (fabs(angle) >= 1e-6) {
    result = 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotationBetween2) //{ */

TEST(TESTSuite, rotationBetween2) {

  int result = 0;

  const Eigen::Vector3d vec1(1, 0, 0);
  const Eigen::Vector3d vec2(0, 0, -10);

  const auto rot_90 = rotationBetween(vec1, vec2);

  double angle = angleBetween(vec1, vec2);

  cout << "Should be 90 deg (angle: " << angle << "):" << std::endl << rot_90 << std::endl;

  if (fabs(angle - (M_PI / 2.0)) <= 1e-6 || (fabs(angle + (M_PI / 2))) <= 1e-6) {
    result = 1;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotationBetween3) //{ */

TEST(TESTSuite, rotationBetween3) {

  int result = 1;

  const Eigen::Vector3d vec1(1, 0, 0);
  const Eigen::Vector3d vec3(1, 0, 1);

  const auto hvec1 = toHomogenous(vec1);

  cout << hvec1 << std::endl;

  const auto rot_45 = rotationBetween(vec1, vec3);

  double angle = angleBetween(vec1, vec3);

  cout << "Should be 45 deg (angle: " << angle << "):" << std::endl << rot_45 << std::endl;

  if (fabs(angle - (M_PI / 4.0)) >= 1e-6) {
    result = 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotationBetween4) //{ */

TEST(TESTSuite, rotationBetween4) {

  int result = 1;

  const Eigen::Vector2d vec4(-5, 0);

  double angle = angleBetween(vec4, vec4);

  cout << "angle from vec4 to vec4 (should be 0): " << angle << std::endl;

  if (fabs(angle) >= 1e-6) {
    result = 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotationBetween5) //{ */

TEST(TESTSuite, rotationBetween5) {

  int result = 0;

  const Eigen::Vector2d vec4(-5, 0);
  const Eigen::Vector2d vec5(10, 0);

  double angle = angleBetween(vec4, vec5);

  cout << "angle from vec4 to vec5 (should be +-pi): " << angle << std::endl;

  if (fabs(angle - M_PI) >= 1e-6 || fabs(angle + M_PI) >= 1e-6) {
    result = 1;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rotationBetween6) //{ */

TEST(TESTSuite, rotationBetween6) {

  int result = 0;

  const Eigen::Vector2d vec4(-5, 0);
  const Eigen::Vector2d vec6(-1, 1);

  double angle = angleBetween(vec4, vec6);

  cout << "angle from vec4 to vec6 (should be -pi/4): " << angle << std::endl;

  if (fabs(angle + (M_PI / 4.0)) <= 1e-6) {
    result = 1;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, wrapAngle2) //{ */

TEST(TESTSuite, wrapAngle2)
{
  const float a = -1;
  const float b = -4;

  const auto ar = radians(a);
  EXPECT_TRUE(approxEqual(ar.value(), 2*M_PI+a));
  const auto br = radians(b);
  EXPECT_TRUE(approxEqual(br.value(), 2*M_PI+b));

  const auto asr = sradians(a);
  EXPECT_EQ(asr.value(), a);
  const auto bsr = sradians(b);
  EXPECT_TRUE(approxEqual(bsr.value(), 2*M_PI+b));

  const double c = -359;
  const double d = -361;
  const double e = 666;
  const double f = 2;

  const auto cd = degrees(c);
  EXPECT_TRUE(approxEqual(cd.value(), 1));
  const auto dd = degrees(d);
  EXPECT_TRUE(approxEqual(dd.value(), 359));
  const auto ed = degrees(e);
  EXPECT_TRUE(approxEqual(ed.value(), 666-360));
  const auto fd = degrees(f);
  EXPECT_TRUE(approxEqual(fd.value(), 2));
}

//}

/* TEST(TESTSuite, addAngle) //{ */

TEST(TESTSuite, addAngle)
{

  for (int i = 0; i < 10000; i++)
  {
    double prev_angle = randd(-10000, 10000);
    double angle      = randd(-10000, 10000);

    double output   = (radians(prev_angle) + radians(angle)).value();
    double expected = wrapAngleTest(addAngleTest(angle, prev_angle), 0.0, 2*M_PI);

    EXPECT_LT(fabs(output - expected), 1e-6);

    output   = (sradians(angle) + sradians(prev_angle)).value();
    expected = wrapAngleTest(addAngleTest(angle, prev_angle), -M_PI, M_PI);

    EXPECT_LT(fabs(output - expected), 1e-6);

    sradians a = prev_angle;
    sradians b(angle);
    sradians c = a+b;

    EXPECT_EQ(c.value(), output);
    EXPECT_EQ((a+angle).value(), output);
    EXPECT_EQ((prev_angle+b).value(), output);
  }
}

//}

/* TEST(TESTSuite, angleOps) //{ */

TEST(TESTSuite, angleOps)
{
  radians var(1);
  EXPECT_TRUE(approxEqual(var, 1));
  var += 2;
  EXPECT_TRUE(approxEqual(var, 3));
  var -= 3;
  EXPECT_TRUE(approxEqual(var, 0));
  var = 4;
  EXPECT_TRUE(approxEqual(var, 4));

  var = var + 3;
  EXPECT_TRUE(approxEqual(var, fmod(4+3, 2*M_PI)));
  var = var + radians(3);
  EXPECT_TRUE(approxEqual(var, fmod(4+3+3, 2*M_PI)));
  var = var + sradians(3).convert<radians>();
  EXPECT_TRUE(approxEqual(var, fmod(4+3+3+3, 2*M_PI)));
  var = var + var;
  EXPECT_TRUE(approxEqual(var, fmod(2*(4+3+3+3), 2*M_PI)));
  var = degrees::convert<radians>(180);
  EXPECT_TRUE(approxEqual(var, M_PI));
}

//}

/* TEST(TESTSuite, diffAngle) //{ */

TEST(TESTSuite, diffAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double prev_angle = randd(-10000, 10000);
    double angle      = randd(-10000, 10000);

    double output   = radians::diff(angle, prev_angle);
    double expected = diffAngleTest(angle, prev_angle);

    EXPECT_EQ(output, radians(angle) - radians(prev_angle));

    if (fabs(output - expected) > 1e-6) {
      printf("diffAngle() #%d failed for radians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sradians::diff(angle, prev_angle);
    expected = wrapAngleTest(diffAngleTest(angle, prev_angle), -M_PI, M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("diffAngle() #%d failed for sradians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, angleCompare) //{ */

TEST(TESTSuite, angleCompare) {

  const float a = -1;
  const float b = -4;

  const auto ar = radians(a);
  const auto br = radians(b);

  const auto asr = sradians(a);
  const auto bsr = sradians(b);

  EXPECT_TRUE(ar < br);
  EXPECT_FALSE(br < ar);

  EXPECT_FALSE(ar > br);
  EXPECT_TRUE(br > ar);

  EXPECT_TRUE(asr < bsr);
  EXPECT_FALSE(bsr < asr);

  EXPECT_FALSE(asr > bsr);
  EXPECT_TRUE(bsr > asr);
}

//}

/* TEST(TESTSuite, distAngle) //{ */

TEST(TESTSuite, distAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double prev_angle = randd(-10000, 10000);
    double angle      = randd(-10000, 10000);

    double output   = radians::dist(angle, prev_angle);
    double expected = distAngleTest(angle, prev_angle);

    if (fabs(output - expected) > 1e-6) {
      printf("distAngle() #%d failed for radians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sradians::dist(angle, prev_angle);
    expected = wrapAngleTest(distAngleTest(angle, prev_angle), -M_PI, M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("distAngle() #%d failed for sradians, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, wrapAngle) //{ */

TEST(TESTSuite, wrapAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double angle = randd(-10000, 10000);

    double output   = radians::wrap(angle);
    double expected = wrapAngleTest(angle, 0.0, 2 * M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sradians::wrap(angle);
    expected = wrapAngleTest(angle, -M_PI, M_PI);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = degrees::wrap(angle);
    expected = wrapAngleTest(angle, 0.0, 360.0);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }

    output   = sdegrees::wrap(angle);
    expected = wrapAngleTest(angle, -180.0, 180.0);

    if (fabs(output - expected) > 1e-6) {
      printf("wrapAngle() #%d faild, input %.2f, output %.2f, expected %.2f\n", i, angle, output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, unwrapAngle) //{ */

TEST(TESTSuite, unwrapAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double previous_angle = randd(-10000, 10000);
    double current_angle  = randd(-10000, 10000);

    double output   = radians::unwrap(current_angle, previous_angle);
    double expected = unwrapAngleTest(current_angle, previous_angle, 2 * M_PI);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for radians, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }

    output   = sradians::unwrap(current_angle, previous_angle);
    expected = unwrapAngleTest(current_angle, previous_angle, 2 * M_PI);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for sradians, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }

    output   = degrees::unwrap(current_angle, previous_angle);
    expected = unwrapAngleTest(current_angle, previous_angle, 360.0);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for degrees, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }

    output   = sdegrees::unwrap(current_angle, previous_angle);
    expected = unwrapAngleTest(current_angle, previous_angle, 360.0);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr, "unwrapAngle() #%d failed for sdegrees, input (prev %.2f, current %.2f), output %.2f, expected %.2f\n", i, previous_angle, current_angle,
              output, expected);
      result = 0;
    }
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, interpolateAngle) //{ */

TEST(TESTSuite, interpolateAngle) {

  int result = 1;

  for (int i = 0; i < 10000; i++) {

    double previous_angle = randd(-10000, 10000);
    double ang_diff       = randd(-M_PI + 1e-9, M_PI - 1e-9);
    double period_diff    = round(randd(-1000, 1000)) * 2 * M_PI;
    double current_angle  = previous_angle + ang_diff + period_diff;  // ensure tha angular difference not M_PI, which would make the solution ambiguous
    double coeff          = randd(-10000, 10000);

    double output   = sradians::interp(previous_angle, current_angle, coeff);
    double expected = interpolateAngleTest(previous_angle, current_angle, coeff);

    if (fabs(output - expected) > 1e-6) {
      fprintf(stderr,
              "interpolateAngle() #%d failed for radians, input (prev %.2frad current %.2frad ~ prev %.2frad, cur %.2frad), output %.2f, expected %.2f\n", i,
              previous_angle, wrapAngleTest(previous_angle), current_angle, wrapAngleTest(current_angle), output, expected);
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
