// clang: MatousFormat
/**  \file
     \brief Example file for the cyclical values implementation (see the cyclic class)
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib cyclic_example`.

     See \ref geometry_utils/cyclic_example.cpp.
 */

/**  \example "geometry_utils/cyclic_example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib cyclic_example`.
 */

// Include the header
#include <mrs_lib/geometry_utils.h>
#include <random>
#include <ros/ros.h>

// Define the cyclic quantity we will be using
namespace mrs_lib
{
  namespace geometry
  {
    struct degrees : public cyclic<double, degrees>
    {
      using cyclic<double, degrees>::cyclic; // necessary to inherit constructors
      static constexpr double minimum = -180;
      static constexpr double supremum = 180;
    };
  }
}

// A helpful aliase to make writing of types shorter
using degrees = mrs_lib::geometry::degrees;

int main()
{
  degrees deg(721.0f); // equal to 1 degree
  std::cout << deg << std::endl;

  return 0;
}


