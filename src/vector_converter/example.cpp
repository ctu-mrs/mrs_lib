// clang: MatousFormat
/**  \file
     \brief Example file for the convert() vector type conversion function
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib vector_converter_example`.

     See \ref vector_converter/example.cpp.
 */

/**  \example "vector_converter/example.cpp"

     This is an example of usage of the convert() function for conversion between different vector types.
     It may be run after building *mrs_lib* by executing `rosrun mrs_lib vector_converter_example`.
 */

#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>

/* Demonstration of usage for custom incompatible types. //{ */

// What if we want to convert from/to a different type, which doesn't provide either operator[], nor x(), y() and z() getter/setter methods, nor x, y and z-named members?
struct MyPoint
{
  double e1, e2, e3;
};

// We need to define our own specialization of the mrs_lib::impl::convertFrom() and mrs_lib::impl::convertTo() functions for this type. convert() will then work automatically.
namespace mrs_lib
{
  namespace impl
  {

    // convertFrom specialization for MyPoint (mrs_lib::impl::unw_t is just a shorthand for std::tuple<double, double, double>)
    std::tuple<double, double, double> convertFrom(const MyPoint& in)
    {
      return {in.e1, in.e2, in.e3};
    }

    // convertTo specialization for MyPoint
    void convertTo(MyPoint& ret, const double x, const double y, const double z)
    {
      ret.e1 = x;
      ret.e2 = y;
      ret.e3 = z;
    }

  }
}

//}

// Include the vector_converter.h header (this has to be included *after* defining functions for your custom types)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#include <mrs_lib/vector_converter.h>
#pragma GCC diagnostic pop

int main()
{
  // prepare a source variable `from` with the type, which we'll want to convert
  const double xo = rand(), yo = rand(), zo = rand();
  // the mrs_lib::impl::convertTo() function is used as a shortcut for initialization of the object
  // Note, then when converting to a single precision floating type,
  // the compiler will emit a narrowing conversion waring. This can be disabled using the pragma statements
  // around the line `#include <mrs_lib/vector_converter.h>` (see above).
  const auto from = mrs_lib::impl::convertTo<pcl::PointXYZ>(xo, yo, zo);

  // This is the main functionality - one-liner for converting between two vector types.
  const auto to = mrs_lib::convert<Eigen::Vector3d>(from);
  const auto to2 = mrs_lib::convert<geometry_msgs::Vector3>(to);

  std::cout << "Converted from type " << typeid(decltype(from)).name() << " with value:" << std::endl << from << std::endl;
  std::cout << "            to type " << typeid(decltype(to)).name() << " with value:" << std::endl << to << std::endl << std::endl;
  std::cout << "   and then to type " << typeid(decltype(to2)).name() << " with value:" << std::endl << to2 << std::endl;

  // Even for custom types with compatible API, the convert() function will work (thanks to templates)
  struct MyPointOK
  {
    double x, y, z;
  };
  const auto custom1 = mrs_lib::convert<MyPointOK>(from); // works
  std::cout << "    and to new type " << typeid(decltype(custom1)).name() << " with value:" << std::endl << custom1.x << std::endl << custom1.y << std::endl << custom1.z << std::endl << std::endl;

  // However, for custom types with incompatible API, the convertFrom() and convertTo() functions have to be manually implemented for convert() to work (take a look before main()):
  const auto custom2 = mrs_lib::convert<MyPoint>(from); // now also works
  std::cout << "    and to new type " << typeid(decltype(custom2)).name() << " with value:" << std::endl << custom2.e1 << std::endl << custom2.e2 << std::endl << custom2.e3 << std::endl << std::endl;
}

