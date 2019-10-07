#include <mrs_lib/geometry_utils.h>

#include <iostream>

using namespace mrs_lib;
using namespace std;

int main()
{

  Eigen::Vector3d vec1(1, 0, 0);
  Eigen::Vector3d vec2(-10, 0, 0);
  Eigen::Vector3d vec3(1, 1, 0);
  const auto rot_iden = rotation_between(vec1, vec1);
  const auto rot_180 = rotation_between(vec1, vec2);
  const auto rot_45 = rotation_between(vec1, vec3);
  cout << "Should be identity:" << std::endl << rot_iden << std::endl;
  cout << "Should be 180 deg rotation:" << std::endl << rot_180 << std::endl;
  cout << "Should be 45 deg rotation:" << std::endl << rot_45 << std::endl;

  return 0;
}
