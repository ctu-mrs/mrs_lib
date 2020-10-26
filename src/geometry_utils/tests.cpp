// clang: MatousFormat

// Include the LKF header
#include <mrs_lib/geometry_utils.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <complex>

using namespace mrs_lib::geometry;

// from https://www.codeproject.com/Articles/190833/Circular-Values-Math-and-Statistics-with-Cplusplus
template<typename T>
T Mod(T x, T y)
{
    static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");

    if (0 == y)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulting from floating-point limited accuracy:

    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14 
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14 
        }
    }

    return m;
}

double wrap(double r)
{
    // the next lines are for optimization and improved accuracy only
    if (r>=radians::minimum)
    {
             if (r< radians::supremum        ) return r        ;
        else if (r< radians::supremum+radians::range) return r-radians::range;
    }
    else
             if (r>=radians::minimum-radians::range) return r+radians::range;
 
    // general case
    return Mod(r - radians::minimum, radians::range) + radians::minimum;
}

double diff(const double a, const double b)
{
  double d = a-b;
  if (d <  -radians::half_range) return d + radians::range;
  if (d >=  radians::half_range) return d - radians::range;
  return d;
}

static double dist(const double a, const double b)
{
    return b>=a ? b-a : radians::range-a+b;
}

int main()
{
  std::function correct(dist);
  std::function totest(radians::dist);

  const std::vector<double> tst {-0.0, 0.0, 1, M_PI, -M_PI, 2*M_PI, -2*M_PI};
  std::cout <<
    "┌─────────┬─────────┬─────────┐\n"
    "│ correct │ mrs_lib │  diff.  │\n"
    "├─────────┼─────────┼─────────┤\n"
    ;
  for (const auto& v1 : tst)
  {
    for (const auto& v2 : tst)
    {
      const auto a = radians::wrap(v1);
      const auto b = radians::wrap(v2);
      const auto cor = correct(a, b);
      const auto res = totest(a, b);
      const auto dif = cor-res;
      const bool ok = dif == 0;
      if (!ok)
        std::cout << "\033[1;31m";
      std::cout << std::left << std::showpos << std::setprecision(4)
      << "│ " << std::setw(7) << cor << " │ " << std::setw(7) << res << " │ " << std::setw(7) << dif << " |";
      if (!ok)
        std::cout << "\tfor values: " << std::left << std::showpos << std::setprecision(4) << v1 << "\tand\t" << v2 << "\033[0m";
      std::cout << std::endl;
    }
  }
  std::cout <<
  "└─────────┴─────────┴─────────┘" << std::endl;

  const auto start1 = std::chrono::high_resolution_clock::now();
  for (int it = 0; it < 1e6; it++)
  {
    for (const auto& v1 : tst)
    {
      for (const auto& v2 : tst)
      {
        const auto a = radians::wrap(v1);
        const auto b = radians::wrap(v2);
        [[maybe_unused]] volatile auto cor = correct(a, b);
      }
    }
  }
  const auto stop1 = std::chrono::high_resolution_clock::now();

  const auto start2 = std::chrono::high_resolution_clock::now();
  for (int it = 0; it < 1e6; it++)
  {
    for (const auto& v1 : tst)
    {
      for (const auto& v2 : tst)
      {
        const auto a = radians::wrap(v1);
        const auto b = radians::wrap(v2);
        [[maybe_unused]] volatile auto cor = totest(a, b);
      }
    }
  }
  const auto stop2 = std::chrono::high_resolution_clock::now();

  std::cout << "dur1: " << std::chrono::duration_cast<std::chrono::microseconds>(stop1-start1).count() << std::endl;
  std::cout << "dur2: " << std::chrono::duration_cast<std::chrono::microseconds>(stop2-start2).count() << std::endl;

  return 0;
}




