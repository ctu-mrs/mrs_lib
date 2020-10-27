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
    struct degrees : public cyclic<float, degrees>
    {
      using cyclic<float, degrees>::cyclic; // necessary to inherit constructors
      static constexpr double minimum = -180;
      static constexpr double supremum = 180;
    };
  }
}

// A few helpful aliases to make writing of types shorter
using degrees = mrs_lib::geometry::degrees;
using radians = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

template <class T>
void printit(const T& a, const std::string& name)
{
  std::cout << name << ":\t" << std::left << std::showpos << std::setprecision(4)
    << a.value() << "\tin [ " << a.minimum << ",\t" << a.supremum << "\t[" << std::endl;
}

int main()
{
  // | ---------- Basic initialization and conversions ---------- |
  degrees a(721.0f);                                // constructor (equal to 1 degree)
  radians b(-M_PI_2);                               // constructor (equal to -45 degrees)
  radians c(b);                                     // copy constructor
  radians d = 65;                                   // assignment operator using primitive types will automatically wrap the value to the valid interval
  radians e = b;                                    // assignment operator (radians class)
  degrees f = b.convert<degrees>();                 // conversion from radians to degrees
  sradians g = b.convert<sradians>();               // conversion from radians to signed radians
  degrees h = radians::convert<degrees>(5*M_PI_2);  // static conversion from radians to degrees

  printit(a, "a");
  printit(b, "b");
  printit(c, "c");
  printit(d, "d");
  printit(e, "e");
  printit(f, "f");
  printit(g, "g");
  printit(h, "h");

  // this is a wrong way of converting radians to degrees - the argument will be interpreted as degrees (the compiler has no way o knowing whether you mean radians or degrees - it's all float to him)
  degrees WRONG1 = degrees(5*M_PI_2);
  // this is forbidden (and will not compile) - use the convert methods for conversions between different types of cyclic quantities
  /* degrees WRONG2 = degrees(b); */

  printit(WRONG1, "WRONG1");
  /* printit(WRONG2, "WRONG2"); */

  // value of the cyclic quantity object can be accessed using a getter
  double adouble = a.value();
  // the minimum, supremum and range are a static property of the class and can be retrieved either from an object of from the class
  double amin = a.minimum;
  double asup = degrees::supremum;
  double aran = degrees::range;
  std::cout << "adouble:" << std::left << std::showpos << std::setprecision(4)
    << adouble << "\tin [ " << amin << ",\t" << asup << "\t[\trange: " << aran << std::endl;

  std::cout << "----------------------------------------------------------------" << std::endl;
  // | -------------------- Binary functions -------------------- |

  return 0;
}


