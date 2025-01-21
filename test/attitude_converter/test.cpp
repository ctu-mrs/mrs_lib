#include <limits>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <cmath>
#include <iostream>
#include <chrono>

#include <gtest/gtest.h>

using namespace mrs_lib;
using namespace std;

double randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

/* TEST(TESTSuite, conversion_chain) //{ */

TEST(TESTSuite, conversion_chain) {

  int result = 1;

  printf("Testing the basic conversion chain:\n");

  double roll  = 0.2;
  double pitch = 0.3;
  double yaw   = 0.8;

  printf("original input: roll=%1.2f, pitch=%1.2f, yaw=%1.2f, heading=%1.2f\n", roll, pitch, yaw, yaw);

  tf2::Quaternion                tf2_quaternion  = AttitudeConverter(roll, pitch, yaw);
  tf2::Matrix3x3                 tf2_matrix      = AttitudeConverter(tf2_quaternion);
  geometry_msgs::msg::Quaternion geom_quaternion = AttitudeConverter(tf2_matrix);
  Eigen::Quaterniond             eig_quaternion  = AttitudeConverter(geom_quaternion);
  Eigen::AngleAxis<double>       eig_angle_axis  = AttitudeConverter(eig_quaternion);
  Eigen::Matrix3d                eig_matrix      = AttitudeConverter(eig_angle_axis);
  EulerAttitude                  euler_angles    = AttitudeConverter(eig_matrix);
  auto [roll2, pitch2, yaw2]                     = AttitudeConverter(euler_angles);
  tie(roll2, pitch2, yaw2)                       = AttitudeConverter(roll2, pitch2, yaw2);
  double roll3                                   = AttitudeConverter(roll2, pitch2, yaw2).getRoll();
  double pitch3                                  = AttitudeConverter(roll2, pitch2, yaw2).getPitch();
  double yaw3                                    = AttitudeConverter(roll2, pitch2, yaw2).getYaw();
  double heading1                                = AttitudeConverter(roll3, pitch3, yaw3).getHeading();

  printf("final output:   roll=%1.2f, pitch=%1.2f, yaw=%1.2f, heading=%1.2f\n", roll3, pitch3, yaw3, heading1);
  printf("\n");

  if (fabs(roll3 - roll) > 1e-6 || fabs(pitch3 - pitch) > 1e-6 || fabs(yaw3 - yaw) > 1e-6) {
    result *= 0;
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, rpy_settters_getters) //{ */

TEST(TESTSuite, rpy_settters_getters) {

  int result = 1;

  printf("Testing intrinsic/extrinsic RPY setters/getters:\n");

  printf("the Z vector should point to [1, 0, 0], \n");

  double e_roll  = M_PI / 2;
  double e_pitch = M_PI / 2;
  double e_yaw   = M_PI / 2;

  double i_roll  = 0;
  double i_pitch = M_PI / 2;
  double i_yaw   = 0;

  tf2::Quaternion quat_from_ext = AttitudeConverter(e_roll, e_pitch, e_yaw, mrs_lib::RPY_EXTRINSIC);  // the default option without the argument
  tf2::Quaternion quat_from_int = AttitudeConverter(i_roll, i_pitch, i_yaw, mrs_lib::RPY_INTRINSIC);

  printf("from extrinsic: quat: [%1.2f, %1.2f, %1.2f, %1.2f]\n", quat_from_ext[0], quat_from_ext[1], quat_from_ext[2], quat_from_ext[3]);
  printf("from intrinsic: quat: [%1.2f, %1.2f, %1.2f, %1.2f]\n", quat_from_int[0], quat_from_int[1], quat_from_int[2], quat_from_int[3]);

  Eigen::Vector3d z_vec_ext = AttitudeConverter(quat_from_ext).getVectorZ();
  Eigen::Vector3d z_vec_int = AttitudeConverter(quat_from_int).getVectorZ();

  printf("from extrinsic: z_vec: [%1.2f, %1.2f, %1.2f]\n", z_vec_ext[0], z_vec_ext[1], z_vec_ext[2]);
  printf("from intrinsic: z_vec: [%1.2f, %1.2f, %1.2f]\n", z_vec_int[0], z_vec_int[1], z_vec_int[2]);

  if (fabs(z_vec_int[0] - z_vec_ext[0]) > 1e-6 || fabs(z_vec_int[1] - z_vec_ext[1]) > 1e-6 || fabs(z_vec_int[2] - z_vec_ext[2]) > 1e-6) {
    printf("z vector's don't match\n");
    result *= 0;
  }

  printf("\n");

  {
    printf("Testing the getters (the rotations should be the same as the input):\n");

    // test quaternion
    tf2::Quaternion quat(0.71, 0.00, 0.71, 0.00);
    quat.normalize();

    Eigen::Matrix3d ref_R = mrs_lib::AttitudeConverter(quat);
    std::cout << "ref R = " << std::endl << ref_R << std::endl;

    Eigen::Vector3d x_vec_ref = AttitudeConverter(quat).getVectorX();
    Eigen::Vector3d y_vec_ref = AttitudeConverter(quat).getVectorY();
    Eigen::Vector3d z_vec_ref = AttitudeConverter(quat).getVectorZ();

    auto [roll, pitch, yaw] = AttitudeConverter(quat).getExtrinsicRPY();
    printf("ext from quat: roll: %1.2f, pitch: %1.2f, yaw: %1.2f\n", roll, pitch, yaw);

    Eigen::Matrix3d ref_ext_R = mrs_lib::AttitudeConverter(roll, pitch, yaw, mrs_lib::RPY_EXTRINSIC);
    std::cout << "ref throught ext R = " << std::endl << ref_ext_R << std::endl;
    ;

    Eigen::Vector3d x_vec = AttitudeConverter(roll, pitch, yaw).getVectorX();
    Eigen::Vector3d y_vec = AttitudeConverter(roll, pitch, yaw).getVectorY();
    Eigen::Vector3d z_vec = AttitudeConverter(roll, pitch, yaw).getVectorZ();

    if (fabs(x_vec_ref[0] - x_vec[0]) > 1e-6 || fabs(x_vec_ref[1] - x_vec[1]) > 1e-6 || fabs(x_vec_ref[2] - x_vec[2]) > 1e-6) {
      printf("x vector's don't match\n");
      result *= 0;
    }

    if (fabs(y_vec_ref[0] - y_vec[0]) > 1e-6 || fabs(y_vec_ref[1] - y_vec[1]) > 1e-6 || fabs(y_vec_ref[2] - y_vec[2]) > 1e-6) {
      printf("y vector's don't match\n");
      result *= 0;
    }

    if (fabs(z_vec_ref[0] - z_vec[0]) > 1e-6 || fabs(z_vec_ref[1] - z_vec[1]) > 1e-6 || fabs(z_vec_ref[2] - z_vec[2]) > 1e-6) {
      printf("z vector's don't match\n");
      result *= 0;
    }

    tie(roll, pitch, yaw) = AttitudeConverter(quat).getIntrinsicRPY();
    printf("int from quat: roll: %1.2f, pitch: %1.2f, yaw: %1.2f\n", roll, pitch, yaw);

    Eigen::Matrix3d ref_int_R = mrs_lib::AttitudeConverter(roll, pitch, yaw, mrs_lib::RPY_INTRINSIC);
    std::cout << "ref throught int R = " << std::endl << ref_int_R << std::endl;
    ;

    x_vec = AttitudeConverter(roll, pitch, yaw, mrs_lib::RPY_INTRINSIC).getVectorX();
    y_vec = AttitudeConverter(roll, pitch, yaw, mrs_lib::RPY_INTRINSIC).getVectorY();
    z_vec = AttitudeConverter(roll, pitch, yaw, mrs_lib::RPY_INTRINSIC).getVectorZ();

    if (fabs(x_vec_ref[0] - x_vec[0]) > 1e-6 || fabs(x_vec_ref[1] - x_vec[1]) > 1e-6 || fabs(x_vec_ref[2] - x_vec[2]) > 1e-6) {
      printf("x vector's don't match\n");
      result *= 0;
    }

    if (fabs(y_vec_ref[0] - y_vec[0]) > 1e-6 || fabs(y_vec_ref[1] - y_vec[1]) > 1e-6 || fabs(y_vec_ref[2] - y_vec[2]) > 1e-6) {
      printf("y vector's don't match\n");
      result *= 0;
    }

    if (fabs(z_vec_ref[0] - z_vec[0]) > 1e-6 || fabs(z_vec_ref[1] - z_vec[1]) > 1e-6 || fabs(z_vec_ref[2] - z_vec[2]) > 1e-6) {
      printf("z vector's don't match\n");
      result *= 0;
    }
  }

  printf("\n");

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, get_heading_exception) //{ */

TEST(TESTSuite, get_heading_exception) {

  int result = 0;

  printf("Testing the getHeading() exception:\n");

  try {
    [[maybe_unused]] double errored_heading = AttitudeConverter(0, M_PI_2, 0).getHeading();
  }
  catch (const mrs_lib::AttitudeConverter::GetHeadingException& e) {
    result = 1;
    printf("exception correctly caught: %s\n", e.what());
  }

  printf("\n");

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, set_heading) //{ */

TEST(TESTSuite, set_heading) {

  int result = 1;

  // | ----------------- test setHeading() ----------------- |

  {
    printf("Testing setHeading():\n");

    for (int i = 0; i < 1000; i++) {

      double roll  = randd(-3.14, 3.14);
      double pitch = randd(-3.14, 3.14);
      double yaw   = randd(-3.14, 3.14);

      tf2::Quaternion old_attitude = AttitudeConverter(roll, pitch, yaw);

      double old_heading;

      try {
        old_heading = AttitudeConverter(old_attitude).getHeading();
      }
      catch (const mrs_lib::AttitudeConverter::GetHeadingException& e) {
        continue;
      }

      Eigen::Vector3d old_z_vec = AttitudeConverter(old_attitude).getVectorZ();

      printf("original input: heading: %1.2f, yaw: %1.2f, z vec: [%1.2f, %1.2f, %1.2f]\n", old_heading, yaw, old_z_vec[0], old_z_vec[1], old_z_vec[2]);

      {
        double new_desired_heading = randd(-3.14, 3.14);
        printf("setting heading to %1.2f:\n", new_desired_heading);

        tf2::Quaternion new_attitude;

        try {
          new_attitude = AttitudeConverter(old_attitude).setHeading(new_desired_heading);
        }
        catch (const mrs_lib::AttitudeConverter::SetHeadingException& e) {
          continue;
        }

        double new_heading;

        try {
          new_heading = AttitudeConverter(new_attitude).getHeading();
        }
        catch (const mrs_lib::AttitudeConverter::GetHeadingException& e) {
          continue;
        }

        Eigen::Vector3d new_z_vec = AttitudeConverter(new_attitude).getVectorZ();

        printf("results: heading: %1.2f, z vec: [%1.2f, %1.2f, %1.2f]\n", new_heading, new_z_vec[0], new_z_vec[1], new_z_vec[2]);

        if (fabs(new_heading - new_desired_heading) > 1e-3) {
          printf("new heading does not match!\n");
          result *= 0;
        }

        if (fabs(new_z_vec[0] - old_z_vec[0]) > 1e-3 || fabs(new_z_vec[1] - old_z_vec[1]) > 1e-3 || fabs(new_z_vec[2] - old_z_vec[2]) > 1e-3) {
          printf("new z vec does not match!");
          result *= 0;
        }
      }
    }

    printf("\n");
  }

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, get_heading_rate) //{ */

TEST(TESTSuite, get_heading_rate) {

  int result = 1;

  printf("checking getHeadingRate() and getYawRateIntrinsic()\n");

  // initial orientation
  Eigen::Matrix3d R = AttitudeConverter(0.2, 1.0, 0.8);

  // attitude rate in the body frame
  geometry_msgs::msg::Vector3 w     = Vector3Converter(0, 0, 0.9);
  Eigen::Vector3d             w_eig = Vector3Converter(w);

  double dt  = 0.1;  // [s]
  int    len = int(round(2 * M_PI / dt));

  double heading = AttitudeConverter(R).getHeading();

  double max_heading_rate_error = 0;
  double max_yaw_rate_error     = 0;

  for (int i = 0; i < len; i++) {

    // create the angular velocity tensor
    Eigen::Matrix3d W;
    W << 0, -w_eig[2], w_eig[1], w_eig[2], 0, -w_eig[0], -w_eig[1], w_eig[0], 0;

    // R derivative
    Eigen::Matrix3d R_d = R * W;

    // update R
    R = R + R_d * dt;

    // normalize R
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d                   U = svd.matrixU();
    Eigen::Matrix3d                   V = svd.matrixV();

    R = U * V.transpose();

    // calculate the heading difference
    double new_heading        = AttitudeConverter(R).getHeading();
    double heading_difference = mrs_lib::geometry::sradians::diff(new_heading, heading);

    // calculate the heading rate analythically
    double heading_rate = AttitudeConverter(R).getHeadingRate(w);

    double heading_rate_error = fabs((heading_difference / dt) - heading_rate);

    // reconstruct a yaw rate from the heading rate
    double yaw_rate = AttitudeConverter(R).getYawRateIntrinsic(heading_rate);

    printf("heading: (%.2f->%.2f), heading rate: estimated %.2f, calcualted: %.2f, difference: %.2f, yaw_rate: %.2f\n", heading, new_heading,
           heading_difference / dt, heading_rate, fabs((heading_difference / dt) - heading_rate), yaw_rate);

    if (fabs(heading_rate_error) > max_heading_rate_error) {
      max_heading_rate_error = fabs(heading_rate_error);
    }

    printf("yaw rate: %.2f\n", yaw_rate);

    // compare with the original set intrinsic yaw rate
    double yaw_rate_error = fabs(yaw_rate - w.z);

    if (fabs(yaw_rate_error) > max_yaw_rate_error) {
      max_yaw_rate_error = fabs(yaw_rate_error);
    }

    heading = new_heading;
  }

  printf("len: %d, max hdg rate error %.2f (%s), max yaw rate error %.2f (%s)\n", len, max_heading_rate_error, max_heading_rate_error > 0.1 ? "BAD" : "GOOD",
         max_yaw_rate_error, max_yaw_rate_error > 0.1 ? "BAD" : "GOOD");

  if (fabs(max_heading_rate_error) > 0.1 || fabs(max_yaw_rate_error) > 0.1) {
    result *= 0;
  }

  printf("\n");

  EXPECT_TRUE(result);
}

//}

/* TEST(TESTSuite, input_nan_checks) //{ */

TEST(TESTSuite, input_nan_checks) {

  int result = 1;

  printf("testing NaNs on input\n");

  double mynan = std::numeric_limits<double>::quiet_NaN();

  tf2::Quaternion q(0, 0, 0, 1);

  // ok by default
  double heading = 0;

  try {
    // this should cause an exception, therefore, q should be ok after this
    q = AttitudeConverter(mynan, 0, 0);
  }
  catch (const mrs_lib::AttitudeConverter::InvalidAttitudeException& e) {
    printf("exception caught: '%s'\n", e.what());
  }

  // this should not be triggered
  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) || !std::isfinite(q.w())) {
    printf("NaNs in the quaternion\n");
    result = 0;  // make the test fail
  }

  try {
    // this should cause an exception, therefore, heading should be ok after this
    heading = AttitudeConverter(mynan, 0, 0).getHeading();
  }
  catch (const mrs_lib::AttitudeConverter::InvalidAttitudeException& e) {
    printf("exception caught: '%s'\n", e.what());
  }

  // this should not pass
  if (!std::isfinite(heading)) {
    printf("NaNs in heading\n");
    result = 0;  // make the test fail
  }

  printf("\n");

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  srand(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

  return RUN_ALL_TESTS();
}
