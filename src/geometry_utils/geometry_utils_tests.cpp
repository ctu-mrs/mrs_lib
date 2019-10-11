#include <mrs_lib/geometry_utils.h>

#include <iostream>

using namespace mrs_lib;
using namespace std;

int main()
{

  const Eigen::Vector3d vec1(1, 0, 0);
  const Eigen::Vector3d vec2(0, 0, -10);
  const Eigen::Vector3d vec3(1, 0, 1);
  const auto hvec1 = to_homogenous(vec1);
  cout << hvec1 << std::endl;
  const auto rot_iden = rotation_between(vec1, vec1);
  const auto rot_180 = rotation_between(vec1, vec2);
  const auto rot_45 = rotation_between(vec1, vec3);
  cout << "Should be identity (angle: " << angle_between(vec1, vec1) << "):" << std::endl << rot_iden << std::endl;
  cout << "Should be 180 deg (angle: " << angle_between(vec1, vec2) << "):" << std::endl << rot_180 << std::endl;
  cout << "Should be 45 deg (angle: " << angle_between(vec1, vec3) << "):" << std::endl << rot_45 << std::endl;

  const Eigen::Vector2d vec4(-5, 0);
  const Eigen::Vector2d vec5(10, 0);
  const Eigen::Vector2d vec6(-1, 1);
  cout << "angle from vec4 to vec4 (should be 0): " << angle_between(vec4, vec4) << std::endl;
  cout << "angle from vec4 to vec5 (should be +-pi): " << angle_between(vec4, vec5) << std::endl;
  cout << "angle from vec4 to vec6 (should be -pi/4): " << angle_between(vec4, vec6) << std::endl;

  return 0;
}
