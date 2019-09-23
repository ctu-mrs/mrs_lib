#include <mrs_lib/geometry_utils.h>

#include <iostream>

using namespace mrs_lib;
using namespace std;

int main()
{

  Eigen::Vector3d vec1(1, 0, 0);
  Eigen::Vector3d vec2(-1, 0, 0);
  const auto rot = rotation_between(vec1, vec2);
  cout << rot << std::endl;

  return 0;
}
