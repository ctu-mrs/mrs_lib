// clang: MatousFormat

// Include the LKF header
#include <mrs_lib/geometry_utils.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <complex>

using namespace mrs_lib::geometry;
using cx = std::complex<double>;

double wrap(double r)
{
  return std::arg(std::polar<double>(1, r));
}

double diff(const double a, const double b)
{
  return std::arg(std::polar<double>(1, a) * std::conj(std::polar<double>(1, b)));
}

static double dist(const double a, const double b)
{
  return std::abs(diff(a, b));
}

int main()
{
  std::function correct(dist);
  auto totest = std::bind<double(double, double)>(radians::dist, std::placeholders::_1, std::placeholders::_2);

  const std::vector<double> tst {-0.0, 0.0, 1, -1, M_PI, -M_PI, 2*M_PI, -2*M_PI};
  std::cout <<
    "┌────────────┬────────────┬────────────┐\n"
    "│  correct   │  mrs_lib   │ difference │\n"
    "├────────────┼────────────┼────────────┤\n"
    ;
  for (const auto& a : tst)
  {
    for (const auto& b : tst)
    {
      const auto cor = correct(a, b);
      const auto res = totest(a, b);
      const auto dif = dist(cor, res);
      const bool ok = dif < 1e-9;
      if (!ok)
        std::cout << "\033[1;31m";
      std::cout << std::left << std::showpos << std::setprecision(4)
      << "│ " << std::setw(10) << cor << " │ " << std::setw(10) << res << " │ " << std::setw(10) << dif << " |";
      if (!ok)
        std::cout << "\tfor values: " << std::left << std::showpos << std::setprecision(4) << a << "\tand\t" << b << "\033[0m";
      std::cout << std::endl;
    }
  }
  std::cout <<
  "└────────────┴────────────┴────────────┘" << std::endl;

  constexpr int N = 1e4;
  const auto start1 = std::chrono::high_resolution_clock::now();
  for (int it = 0; it < N; it++)
  {
    for (const auto& a : tst)
    {
      for (const auto& b : tst)
      {
        [[maybe_unused]] volatile auto cor = correct(a, b);
      }
    }
  }
  const auto stop1 = std::chrono::high_resolution_clock::now();

  const auto start2 = std::chrono::high_resolution_clock::now();
  for (int it = 0; it < N; it++)
  {
    for (const auto& a : tst)
    {
      for (const auto& b : tst)
      {
        [[maybe_unused]] volatile auto cor = totest(a, b);
      }
    }
  }
  const auto stop2 = std::chrono::high_resolution_clock::now();

  std::cout << "dur1: " << std::chrono::duration_cast<std::chrono::microseconds>(stop1-start1).count()/double(N) << "us" << std::endl;
  std::cout << "dur2: " << std::chrono::duration_cast<std::chrono::microseconds>(stop2-start2).count()/double(N) << "us" << std::endl;

  return 0;
}




