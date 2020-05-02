// clang: TomasFormat

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry_utils.h>

#include <iostream>

using namespace mrs_lib;
using namespace std;

int main() {

  // | -------- test the basic chain with all the formats ------- |

  {
    printf("Testing the basic conversion chain:\n");

    double roll  = 0.2;
    double pitch = 0.3;
    double yaw   = 0.8;

    printf("original input: roll=%1.2f, pitch=%1.2f, yaw=%1.2f, heading=%1.2f\n", roll, pitch, yaw, yaw);

    tf::Quaternion            tf1_quaternion  = AttitudeConverter(roll, pitch, yaw);
    tf2::Quaternion           tf2_quaternion  = AttitudeConverter(tf1_quaternion);
    tf2::Matrix3x3            tf2_matrix      = AttitudeConverter(tf2_quaternion);
    geometry_msgs::Quaternion geom_quaternion = AttitudeConverter(tf2_matrix);
    Eigen::Quaterniond        eig_quaternion  = AttitudeConverter(geom_quaternion);
    Eigen::AngleAxis<double>  eig_angle_axis  = AttitudeConverter(eig_quaternion);
    Eigen::Matrix3d           eig_matrix      = AttitudeConverter(eig_angle_axis);
    EulerAttitude             euler_angles    = AttitudeConverter(eig_matrix);
    auto [roll2, pitch2, yaw2]                = AttitudeConverter(euler_angles);
    tie(roll2, pitch2, yaw2)                  = AttitudeConverter(roll2, pitch2, yaw2);
    double roll3                              = AttitudeConverter(roll2, pitch2, yaw2).getRoll();
    double pitch3                             = AttitudeConverter(roll2, pitch2, yaw2).getPitch();
    double yaw3                               = AttitudeConverter(roll2, pitch2, yaw2).getYaw();
    double heading1                           = AttitudeConverter(roll3, pitch3, yaw3).getHeading();

    printf("final output:   roll=%1.2f, pitch=%1.2f, yaw=%1.2f, heading=%1.2f\n", roll3, pitch3, yaw3, heading1);
    printf("\n");
  }

  // | ------- test the intrinsic/extrinsic setter/getter ------- |

  {

    printf("Testing intrinsic/extrinsic RPY setters/getters:\n");

    printf("the Z vector should point to [1, 0, 0], \n");

    double e_roll  = 1.57;
    double e_pitch = 1.57;
    double e_yaw   = 1.57;

    double i_roll  = 0;
    double i_pitch = 1.57;
    double i_yaw   = 0;

    tf2::Quaternion quat_from_ext = AttitudeConverter(e_roll, e_pitch, e_yaw, mrs_lib::RPY_EXTRINSIC);  // the default option without the argument
    tf2::Quaternion quat_from_int = AttitudeConverter(i_roll, i_pitch, i_yaw, mrs_lib::RPY_INTRINSIC);

    printf("from extrinsic: quat: [%1.2f, %1.2f, %1.2f, %1.2f]\n", quat_from_ext[0], quat_from_ext[1], quat_from_ext[2], quat_from_ext[3]);
    printf("from intrinsic: quat: [%1.2f, %1.2f, %1.2f, %1.2f]\n", quat_from_int[0], quat_from_int[1], quat_from_int[2], quat_from_int[3]);

    Eigen::Vector3d z_vec_ext = AttitudeConverter(quat_from_ext).getVectorZ();
    Eigen::Vector3d z_vec_int = AttitudeConverter(quat_from_int).getVectorZ();

    printf("from extrinsic: z_vec: [%1.2f, %1.2f, %1.2f]\n", z_vec_ext[0], z_vec_ext[1], z_vec_ext[2]);
    printf("from intrinsic: z_vec: [%1.2f, %1.2f, %1.2f]\n", z_vec_int[0], z_vec_int[1], z_vec_int[2]);

    printf("\n");

    printf("Testing the getters (the rotations should be the same as the input):\n");
    auto [roll, pitch, yaw] = AttitudeConverter(quat_from_int).getExtrinsicRPY();
    printf("ext from int: roll: %1.2f, pitch: %1.2f, yaw: %1.2f\n", roll, pitch, yaw);
    tie(roll, pitch, yaw) = AttitudeConverter(quat_from_ext).getIntrinsicRPY();
    printf("int from ext: roll: %1.2f, pitch: %1.2f, yaw: %1.2f\n", roll, pitch, yaw);

    printf("\n");
  }

  // | ------------- test the getHeading() exception ------------ |

  {

    printf("Testing the getHeading() exception:\n");

    try {
      [[maybe_unused]] double errored_heading = AttitudeConverter(0, M_PI_2, 0).getHeading();
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException e) {
      printf("exception correctly caught: %s\n", e.what());
    }

    printf("\n");
  }

  // | ----------------- test setHeadingByYaw() ----------------- |

  {
    printf("Testing setHeadingByYaw():\n");

    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;

    tf2::Quaternion old_attitude = AttitudeConverter(roll, pitch, yaw);

    double          old_heading = AttitudeConverter(old_attitude).getHeading();
    Eigen::Vector3d old_z_vec   = AttitudeConverter(old_attitude).getVectorZ();

    printf("original input: heading: %1.2f, yaw: %1.2f, z vec: [%1.2f, %1.2f, %1.2f]\n", old_heading, yaw, old_z_vec[0], old_z_vec[1], old_z_vec[2]);

    {
      double new_desired_heading = -1.58;
      printf("setting heading to %1.2f:\n", new_desired_heading);
      printf("this is the propper way how to update the heading part of an existing orientation without chaning the Z axis direction\n");

      tf2::Quaternion new_attitude = AttitudeConverter(old_attitude).setHeadingByYaw(new_desired_heading);
      double          new_heading  = AttitudeConverter(new_attitude).getHeading();

      Eigen::Vector3d new_z_vec = AttitudeConverter(new_attitude).getVectorZ();
      double          new_yaw   = AttitudeConverter(new_attitude).getYaw();

      printf("results:        heading: %1.2f, yaw: %1.2f, z vec: [%1.2f, %1.2f, %1.2f]\n", new_heading, new_yaw, new_z_vec[0], new_z_vec[1], new_z_vec[2]);
    }

    {
      double new_desired_yaw = -1.57;
      printf("setting yaw to %1.2f:\n", new_desired_yaw);
      printf("this should not work, setting yaw directly will result in change of the Z axis direction\n");

      tf2::Quaternion new_attitude = AttitudeConverter(old_attitude).setYaw(new_desired_yaw);
      double          new_heading  = AttitudeConverter(new_attitude).getHeading();

      Eigen::Vector3d new_z_vec = AttitudeConverter(new_attitude).getVectorZ();
      double          new_yaw   = AttitudeConverter(new_attitude).getYaw();

      printf("results:        heading: %1.2f, yaw: %1.2f, z vec: [%1.2f, %1.2f, %1.2f]\n", new_heading, new_yaw, new_z_vec[0], new_z_vec[1], new_z_vec[2]);
    }

    printf("\n");
  }

  // | ---------- test the setHeadingByYaw() exception ---------- |

  {
    printf("Testing the setHeading() exception:\n");

    try {
      [[maybe_unused]] tf2::Quaternion attitude = AttitudeConverter(0, M_PI_2, 0).setHeadingByYaw(1.0);
    }
    catch (mrs_lib::AttitudeConverter::SetHeadingByYawException e) {
      printf("exception correctly caught: %s\n", e.what());
    }

    printf("\n");
  }

  // | ------------------ test getHeadingRate() ----------------- |
  {

    printf("checking getHeadingRate() and getYawRateIntrinsic()\n");

    // initial orientation
    Eigen::Matrix3d R = AttitudeConverter(0, 1.0, 0);

    // attitude rate in the body frame
    geometry_msgs::Vector3 w     = Vector3Converter(0, 0, 1);
    Eigen::Vector3d        w_eig = Vector3Converter(w);

    double dt  = 0.1;  // [s]
    int    len = 2 * M_PI / dt;

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
      double heading_difference = mrs_lib::angleBetween(new_heading, heading);

      // calculate the heading rate analythically
      double heading_rate = AttitudeConverter(R).getHeadingRate(w);

      double heading_rate_error = fabs((heading_difference / dt) - heading_rate);

      // reconstruct a yaw rate from the heading rate
      double yaw_rate = AttitudeConverter(R).getYawRateIntrinsic(heading_rate);

      printf("heading: (%.2f->%.2f), heading rate: estimated %.2f, calcualted: %.2f, difference: %.2f, yaw_rate: %.2f\n", heading, new_heading, heading_difference / dt,
             heading_rate, fabs((heading_difference / dt) - heading_rate), yaw_rate);

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

    printf("\n");
  }

  return 0;
}
