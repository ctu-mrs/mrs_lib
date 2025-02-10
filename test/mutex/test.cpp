#include <gtest/gtest.h>

#include <mrs_lib/mutex.h>

#include <string>

/* TEST(TESTSuite, set_mutexed_single) //{ */

TEST(TESTSuite, set_mutexed_single) {

  std::mutex mutex_;

  int prev_int = 333;
  int new_int  = 666;

  mrs_lib::set_mutexed(mutex_, prev_int, new_int);

  ASSERT_EQ(prev_int, new_int);
}

//}

/* TEST(TESTSuite, set_mutexed_multi) //{ */

TEST(TESTSuite, set_mutexed_multi) {

  std::mutex mutex_;

  int prev_int = 333;
  int new_int  = 666;

  bool prev_bool = true;
  bool new_bool  = false;

  std::string prev_str = "pes";
  std::string new_str  = "kocka";

  mrs_lib::set_mutexed(mutex_, new_int, prev_int, new_bool, prev_bool, new_str, prev_str);

  ASSERT_EQ(new_int, 666);
  ASSERT_EQ(new_bool, false);
  ASSERT_EQ(new_str, "kocka");

  ASSERT_EQ(prev_int, 666);
  ASSERT_EQ(prev_bool, false);
  ASSERT_EQ(prev_str, "kocka");
}

//}

/* TEST(TESTSuite, set_mutexed_multi_tupled) //{ */

TEST(TESTSuite, set_mutexed_multi_tupled) {

  std::mutex mutex_;

  int prev_int = 333;
  int new_int  = 666;

  bool prev_bool = true;
  bool new_bool  = false;

  mrs_lib::set_mutexed(mutex_, std::tuple(new_int, new_bool), std::forward_as_tuple(prev_int, prev_bool));

  ASSERT_EQ(new_int, 666);
  ASSERT_EQ(new_bool, false);

  ASSERT_EQ(prev_int, 666);
  ASSERT_EQ(prev_bool, false);
}

//}

/* TEST(TESTSuite, get_mutexed) //{ */

TEST(TESTSuite, get_mutexed) {

  std::mutex mutex_;

  int prev_int  = 333;
  int prev_bool = true;

  auto [cur_int, cur_bool] = mrs_lib::get_mutexed(mutex_, prev_int, prev_bool);

  ASSERT_EQ(cur_int, prev_int);
  ASSERT_EQ(cur_bool, prev_bool);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
