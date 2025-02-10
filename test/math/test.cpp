#include <mrs_lib/math.h>
#include <cmath>

#include <gtest/gtest.h>

#include "probit_gts.h"

using namespace mrs_lib;
using namespace std;

/* TEST(TESTSuite, inv_cdf) //{ */

TEST(TESTSuite, inv_cdf)
{
  const double eps = 2e-5;

  for (int it = 0; it < n_samples; it++)
  {
    const double in = probit_ins[it];
    const double gt = probit_gts[it];
    const double val = mrs_lib::probit(in);
    if (in == 0.0 || in == 1.0)
      EXPECT_EQ(gt, val);
    else
      EXPECT_LT(fabs(gt-val), eps);
  }

  EXPECT_TRUE(std::isnan(mrs_lib::probit(-0.1)));
  EXPECT_TRUE(std::isnan(mrs_lib::probit(1.1)));
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
