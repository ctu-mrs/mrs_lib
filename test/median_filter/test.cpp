#include <mrs_lib/median_filter.h>
#include <cmath>

#include <gtest/gtest.h>

using namespace mrs_lib;
using namespace std;

/* randd() //{ */

double randd(double from, double to)
{
  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

//}

/* TEST(TESTSuite, constraints) //{ */

TEST(TESTSuite, constraints)
{
  constexpr size_t bfr_len = 5;
  constexpr double min = 0;
  constexpr double max = 10;
  constexpr double max_diff = 5;
  mrs_lib::MedianFilter fil(bfr_len, min, max, max_diff);

  // doesn't comply with min constraint
  EXPECT_FALSE(fil.check(-1));
  // doesn't comply with max constraint
  EXPECT_FALSE(fil.check(11));

  // fine
  EXPECT_TRUE(fil.addCheck(1));
  // now, the median value should be 1
  EXPECT_EQ(fil.median(), 1);
  // doesn't comply with max. diff. constraint
  EXPECT_FALSE(fil.check(9));

  // fine
  EXPECT_TRUE(fil.addCheck(3));
  // now, the median value should be 2
  EXPECT_EQ(fil.median(), 2);

  fil.setMaxValue(20);
  // should still fail due to max. threshold
  EXPECT_FALSE(fil.check(11));

  fil.setMaxDifference(10);
  // now should be fine
  EXPECT_TRUE(fil.addCheck(11));
  // now, the median value should be 3
  EXPECT_EQ(fil.median(), 3);

  fil.setMinValue(-10);
  // now should be fine
  EXPECT_TRUE(fil.addCheck(-1));
  // now, the median value should be 2 again
  EXPECT_EQ(fil.median(), 2);
}

//}

/* TEST(TESTSuite, median) //{ */

TEST(TESTSuite, median)
{
  constexpr size_t bfr_len = 5;
  mrs_lib::MedianFilter fil(bfr_len);

  EXPECT_TRUE(std::isnan(fil.median()));
  EXPECT_TRUE(fil.addCheck(0));
  EXPECT_EQ(fil.median(), 0);
  EXPECT_TRUE(fil.addCheck(1));
  EXPECT_EQ(fil.median(), 0.5);
  EXPECT_TRUE(fil.addCheck(2));
  EXPECT_EQ(fil.median(), 1);
  EXPECT_TRUE(fil.addCheck(3));
  EXPECT_EQ(fil.median(), 1.5);
  EXPECT_TRUE(fil.addCheck(4));
  EXPECT_EQ(fil.median(), 2);
  EXPECT_TRUE(fil.addCheck(5));
  EXPECT_EQ(fil.median(), 3);
  EXPECT_TRUE(fil.addCheck(100));
  EXPECT_EQ(fil.median(), 4);

  fil.clear();
  EXPECT_TRUE(std::isnan(fil.median()));
  EXPECT_TRUE(fil.addCheck(666));
  EXPECT_EQ(fil.median(), 666);
}

//}

/* TEST(TESTSuite, full) //{ */

TEST(TESTSuite, full)
{
  constexpr size_t bfr_len = 5;
  mrs_lib::MedianFilter fil(bfr_len);

  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(1));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(2));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(3));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(4));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(5));
  EXPECT_TRUE(fil.full());
  fil.clear();
  EXPECT_FALSE(fil.full());

  EXPECT_TRUE(fil.addCheck(1));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(2));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(3));
  EXPECT_FALSE(fil.full());
  fil.setBufferLength(3);
  EXPECT_TRUE(fil.full());
  fil.setBufferLength(5);
  EXPECT_FALSE(fil.full());
}

//}

/* TEST(TESTSuite, copy) //{ */

TEST(TESTSuite, copy)
{
  constexpr size_t bfr_len = 5;
  mrs_lib::MedianFilter fil(bfr_len);

  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(1));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(2));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(3));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(4));
  EXPECT_FALSE(fil.full());
  EXPECT_TRUE(fil.addCheck(5));
  EXPECT_TRUE(fil.full());
  EXPECT_EQ(fil.median(), 3);

  {
    // copy constructor
    mrs_lib::MedianFilter fil2 = fil;
    EXPECT_TRUE(fil2.full());
    EXPECT_EQ(fil2.median(), 3);
  }

  {
    // move constructor
    mrs_lib::MedianFilter fil2(std::move(fil));
    EXPECT_TRUE(fil2.full());
    EXPECT_EQ(fil2.median(), 3);
  }

  {
    // empty constructor that will construct an object with a zero-length buffer
    mrs_lib::MedianFilter fil2;
    EXPECT_FALSE(fil2.initialized());
    EXPECT_TRUE(fil2.full());
    fil2.setMinValue(-100);
    fil2.setMaxValue(100);
    fil2.setMaxDifference(200);
    EXPECT_TRUE(std::isnan(fil2.median()));
    EXPECT_TRUE(fil2.addCheck(1));
    EXPECT_TRUE(std::isnan(fil2.median()));
  }
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
