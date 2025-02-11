#include <mrs_lib/msg_extractor.h>

#include <iostream>

#include <chrono>

#include <gtest/gtest.h>

using namespace mrs_lib;
using namespace std;

double randd(const double from, const double to) {

  double zero_to_one = double((float)rand()) / double(RAND_MAX);

  return floor(to - from) * zero_to_one + from;
}

/* TEST(TESTSuite, basic) //{ */

TEST(TESTSuite, basic) {

}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
