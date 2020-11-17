// clang: MatousFormat
/**  \file
     \brief Example file for the cyclical values implementation (see the cyclic class)
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib cyclic_example`.

     See \ref geometry/cyclic_example.cpp.
 */

/**  \example "geometry/cyclic_example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib cyclic_example`.
 */

// Include the header
#include <mrs_lib/geometry/cyclic.h>
#include <random>
#include <ros/ros.h>

// Define the cyclic quantity we will be using (just a float redefinition of degrees)
struct degrees : public mrs_lib::geometry::cyclic<float, degrees>
{
  using cyclic<float, degrees>::cyclic;  // necessary to inherit constructors
  static constexpr double minimum = -180;
  static constexpr double supremum = 180;
};

// A few helpful aliases to make writing of types shorter
using radians = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

/* helper printing functions //{ */

template <class T>
void printit(const T& a, const std::string& name)
{
  std::cout << name << ":\t" << std::left << std::showpos << std::setprecision(4) << a.value() << "\tin [ " << a.minimum << ",\t" << a.supremum << "\t["
            << std::endl;
}

template <class T>
void printcont(const T& cont, const std::string& name)
{
  std::cout << name << ":\t";
  for (const auto& el : cont)
    std::cout << el << "\t";
  std::cout << std::endl;
}

//}

int main()
{
  // | ---------- Basic initialization and conversions ---------- |

  /* initialization, assignment, conversion //{ */

  degrees a(721.0f);                                  // constructor (equal to 1 degree)
  radians b(-M_PI_2);                                 // constructor (equal to -45 degrees)
  radians c(b);                                       // copy constructor
  radians d = 65;                                     // assignment operator using primitive types will automatically wrap the value to the valid interval
  radians e = b;                                      // assignment operator (radians class)
  degrees f = b.convert<degrees>();                   // conversion from radians to degrees
  sradians g = b.convert<sradians>();                 // conversion from radians to signed radians
  degrees h = radians::convert<degrees>(5 * M_PI_2);  // static conversion from radians to degrees

  printit(a, "a");
  printit(b, "b");
  printit(c, "c");
  printit(d, "d");
  printit(e, "e");
  printit(f, "f");
  printit(g, "g");
  printit(h, "h");

  //}

  /* wrong way of converting //{ */

  // this is a wrong way of converting radians to degrees - the argument will be interpreted as degrees (the compiler has no way o knowing whether you mean
  // radians or degrees - it's all float to him)
  degrees WRONG1 = degrees(5 * M_PI_2);
  // this is forbidden (and will not compile) - use the convert methods for conversions between different types of cyclic quantities
  /* degrees WRONG2 = degrees(b); */

  printit(WRONG1, "WRONG1");
  /* printit(WRONG2, "WRONG2"); */

  //}

  /* accessors //{ */

  // value of the cyclic quantity object can be accessed using a getter
  double adouble = a.value();
  // the minimum, supremum and range are a static property of the class and can be retrieved either from an object of from the class
  double amin = a.minimum;
  double asup = degrees::supremum;
  double aran = degrees::range;
  std::cout << "adouble:" << std::left << std::showpos << std::setprecision(4) << adouble << "\tin [ " << amin << ",\t" << asup << "\t[\trange: " << aran
            << std::endl;

  //}

  std::cout << "----------------------------------------------------------------" << std::endl;
  // | -------------------- Utility functions ------------------- |

  /* wrapping //{ */

  // you can check whether a value is in the valid range of a type
  double ang = 666;
  if (!degrees::inRange(ang))
  {
    std::cout << std::left << std::showpos << std::setprecision(4) << ang << "\tis not within [ " << degrees::minimum << ",\t" << degrees::supremum << "\t["
              << std::endl;
    // and wrap it to the correct range if needed
    double wang = degrees::wrap(ang);
    std::cout << std::left << std::showpos << std::setprecision(4) << ang << "\tafter wrapping: " << wang << std::endl;
  }

  //}

  /* unwrapping //{ */

  // if you have a sequence of cyclic values and want to "linearize" it (eg. for a Kalman filter or smh, idk), you can use the unwrap function
  std::vector<double> data = {0, 61, 122, 183, 244, 305, 6, 67, 128, 189};
  printcont(data, "data before unwrapping");

  for (size_t it = 1; it < data.size(); it++)
  {
    const auto prev = data.at(it - 1);
    auto& cur = data.at(it);
    cur = degrees::unwrap(cur, prev);
  }
  printcont(data, "data after unwrapping");

  // and then you can wrap it to the corresponding interval! Go crazy!
  for (auto& el : data)
    el = degrees::wrap(el);
  printcont(data, "data after wrapping");

  // AND UNWRAP IT AGAIN BECAUSE IT WORKS LIKE MAGIC!!!
  for (size_t it = 1; it < data.size(); it++)
  {
    const auto prev = data.at(it - 1);
    auto& cur = data.at(it);
    cur = degrees::unwrap(cur, prev);
  }
  printcont(data, "data after unwrapping");

  //}

  std::cout << "----------------------------------------------------------------" << std::endl;
  // | ------------------- Distance functions ------------------- |

  /* dist() //{ */

  // there are three distance-related functions provided, each with its own uses
  // firstly, there is the dist() function, which simply returns the shortest distance from A to B in the cyclic sense
  degrees A(+33.3);
  degrees B(-33.3);
  std::cout << std::left << std::showpos << std::setprecision(4) << "A:\t" << A.value() << std::endl
            << "B:\t" << B.value() << std::endl
            << "dist(A, B):\t" << degrees::dist(A, B) << std::endl
            << "dist(B, A):\t" << degrees::dist(B, A) << std::endl;
  // notice, that dist() is commutative (perhaps unsurprisingly)

  //}

  /* diff() //{ */

  // secondly, diff() returns the difference between B, subtracted from A
  // this is the same as dist() except that it is signed (the sign depending on the order of the operands)
  degrees C(+146.7);
  degrees D(-146.7);
  std::cout << std::left << std::showpos << std::setprecision(4) << "C:\t" << C.value() << std::endl
            << "D:\t" << D.value() << std::endl
            << "diff(C, D):\t" << degrees::diff(C, D) << std::endl
            << "diff(D, C):\t" << degrees::diff(D, C) << std::endl;
  // naturally, uncomutatitve, and diff(C, D) = -diff(D, C) as you'd expect from subtraction

  //}

  /* pdist() //{ */

  // lastly, there is the pdist function, which returns distance from A to B in a positive direction
  degrees E(+1);
  degrees F(-1);
  std::cout << std::left << std::showpos << std::setprecision(4) << "E:\t" << E.value() << std::endl
            << "F:\t" << F.value() << std::endl
            << "pdist(E, F):\t" << degrees::pdist(E, F) << std::endl
            << "pdist(F, E):\t" << degrees::pdist(F, E) << std::endl;
  // naturally, the result is always positive

  //}

  std::cout << "----------------------------------------------------------------" << std::endl;
  // | ----------------- Interpolation functions ---------------- |

  /* interpolation //{ */

  // it may happen that during your carrier as a human being, you might want to interpolate two angles
  // worry not, I've got your back!
  // this is implemented by the interp() and interpUnwrapped() functions
  // these differ only whether the result is wrapped to the valid interval of values or not (which might be sometimes useful and is sliiightly faster)
  // BEHOLD:
  double a1 = -359;
  double a2 = -2;
  double coeff = 1.0 / 3.0;
  std::cout << std::left << std::showpos << std::setprecision(4) << "a1:\t" << a1 << std::endl
            << "a2:\t" << a2 << std::endl
            << "coeff:\t" << coeff << std::endl
            << "interp(a1, a2, coeff):\t" << degrees::interp(a1, a2, coeff) << std::endl
            << "interpUnwrapped(a1, a2, coeff):\t" << degrees::interpUnwrapped(a1, a2, coeff) << std::endl;

  //}

  return 0;
}

