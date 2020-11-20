#include <mrs_lib/dkf.h>

// Some helpful aliases to make writing of types shorter
using namespace mrs_lib;

int main()
{
  DKF dkf;

  DKF::statecov_t sc{};
  Eigen::Matrix<double, DKF::n, -1> bases(DKF::n, 2);
  bases << 1, 0,
           0, 1,
           0, 0;
  Eigen::Matrix<double, -1, 1> origin(2, 1);
  origin << 6, 6;
  dkf.correct(sc, bases, origin);

  return 0;
}


