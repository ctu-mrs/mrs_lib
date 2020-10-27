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

// A few helpful aliases to make writing of types shorter
using degrees = mrs_lib::geometry::degrees;
using radians = mrs_lib::geometry::radians;

template <class T>
void printit(const T& a, const std::string& name)
{
  std::cout << name << ":\t" << std::left << std::showpos << std::setprecision(4)
    << float(a) << "\tin [ " << a.minimum << ",\t" << a.supremum << "\t[" << std::endl;
}

int main()
{
  degrees a(721.0f);  // equal to 1 degree
  radians b(-M_PI_2); // equal to -45 degrees
  degrees c = radians::convert<degrees>(M_PI_2);
  degrees d = b.convert<degrees>();

  printit(a, "a");
  printit(b, "b");
  printit(c, "c");
  printit(d, "d");

  return 0;
}


