// clang: TomasFormat

#include <mrs_lib/attitude_converter.h>

#include <iostream>

using namespace mrs_lib;
using namespace std;

int main() {

  double roll  = 0.1;
  double pitch = 0.2;
  double yaw   = 0.8;

  tf::Quaternion            tf1_quaternion  = AttitudeConverter(roll, pitch, yaw);
  tf2::Quaternion           tf2_quaternion  = AttitudeConverter(tf1_quaternion);
  geometry_msgs::Quaternion geom_quaternion = AttitudeConverter(tf1_quaternion);
  Eigen::Quaterniond        eig_quaternion  = AttitudeConverter(geom_quaternion);
  Eigen::AngleAxis<double>  eig_angle_axis  = AttitudeConverter(eig_quaternion);
  Eigen::Matrix3d           eig_matrix      = AttitudeConverter(eig_angle_axis);
  EulerAttitude             euler_angles    = AttitudeConverter(eig_matrix);
  auto [roll2, pitch2, yaw2]                = AttitudeConverter(euler_angles);
  tie(roll, pitch, yaw)                     = AttitudeConverter(roll2, pitch2, yaw2);

  std::stringstream ss;
  ss << std::setprecision(2);

  ss << "in: [" << roll << ", " << pitch << ", " << yaw << "]" << endl;
  ss << "tf: [" << tf1_quaternion.x() << ", " << tf1_quaternion.y() << ", " << tf1_quaternion.z() << ", " << tf1_quaternion.w() << "]" << endl;
  ss << "tf2: [" << tf2_quaternion.x() << ", " << tf2_quaternion.y() << ", " << tf2_quaternion.z() << ", " << tf2_quaternion.w() << "]" << endl;
  ss << "geometry_msgs: [" << geom_quaternion.x << ", " << geom_quaternion.y << ", " << geom_quaternion.z << ", " << geom_quaternion.w << "]" << endl;
  ss << "eigen quat: [" << eig_quaternion.x() << ", " << eig_quaternion.y() << ", " << eig_quaternion.z() << ", " << eig_quaternion.w() << "]" << endl;
  ss << "eigen angle_axis: [angle: " << eig_angle_axis.angle() << ", axis: " << eig_angle_axis.axis()[0] << " " << eig_angle_axis.axis()[1] << " "
     << eig_angle_axis.axis()[2] << "]" << endl;
  ss << "rotational matrix: [" << eig_matrix << "]" << endl;
  ss << "euler_angles: [" << euler_angles.roll() << ", " << euler_angles.pitch() << ", " << euler_angles.yaw() << "]" << endl;
  ss << "out: [" << roll2 << ", " << pitch2 << ", " << yaw2 << "]" << endl;

  cout << ss.str();

  return 0;
}
