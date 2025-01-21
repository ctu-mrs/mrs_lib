#include <mrs_lib/utils.h>
#include <cmath>
#include <iostream>
#include <boost/array.hpp>

#include <gtest/gtest.h>

using namespace mrs_lib;
using namespace std;

/* TEST(TESTSuite, container_to_string) //{ */

TEST(TESTSuite, container_to_string) {

  int result = 1;

  std::vector<int> vec = {1, 2, 666};
  std::string      str = mrs_lib::containerToString(std::begin(vec) + 1, std::end(vec), ";");
  std::cout << str << std::endl;

  if (str != "2;666") {
    result *= 0;
  }

  boost::array<int, 3> arr  = {6, 6, 6};
  std::string          str2 = mrs_lib::containerToString(arr, ", ");

  if (str2 != "6, 6, 6") {
    result *= 0;
  }
  std::cout << str2 << std::endl;

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
