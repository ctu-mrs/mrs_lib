#include <gtest/gtest.h>

#include <mrs_lib/lockable.h>

using namespace std::chrono_literals;
using namespace std::string_literals;

class Test : public ::testing::Test {

public:
protected:
};

struct parameters
{
  bool bool_param;
  int int_param;
  float float_param;
  std::string string_param;
};

/* TEST_F(Test, lockable_unlocker) //{ */

TEST_F(Test, lockable_unlocker) {

  // TODO: actual tests of the thread-safe access? How to even do that?
  mrs_lib::Lockable<parameters> params_lck(
      {
        true,
        15,
        666.0f,
        "initial_value"
      });

  EXPECT_EQ(params_lck.unsafe_access().bool_param, true);
  EXPECT_EQ(params_lck.unsafe_access().int_param, 15);
  EXPECT_EQ(params_lck.unsafe_access().float_param, 666.0f);
  EXPECT_EQ(params_lck.unsafe_access().string_param, "initial_value");

  params_lck.unsafe_access().string_param = "unsafe_value";
  EXPECT_EQ(params_lck.unsafe_access().string_param, "unsafe_value");

  params_lck.set_mutexed({false, -15, 333.0f, "mutexed_value"});
  EXPECT_TRUE(params_lck.mutex().try_lock());
  params_lck.mutex().unlock();
  auto vals_copy = params_lck.get_mutexed();
  EXPECT_EQ(params_lck.unsafe_access().bool_param, false);
  EXPECT_EQ(params_lck.unsafe_access().int_param, -15);
  EXPECT_EQ(params_lck.unsafe_access().float_param, 333.0f);
  EXPECT_EQ(params_lck.unsafe_access().string_param, "mutexed_value");
  EXPECT_EQ(vals_copy.bool_param, false);
  EXPECT_EQ(vals_copy.int_param, -15);
  EXPECT_EQ(vals_copy.float_param, 333.0f);
  EXPECT_EQ(vals_copy.string_param, "mutexed_value");

  vals_copy.string_param = "copy_change";
  EXPECT_EQ(params_lck.unsafe_access().string_param, "mutexed_value");

  EXPECT_TRUE(params_lck.mutex().try_lock());
  params_lck.mutex().unlock();
  {
    mrs_lib::Unlocker ulk(params_lck);
    EXPECT_EQ(ulk->bool_param, false);
    EXPECT_EQ(ulk->int_param, -15);
    EXPECT_EQ(ulk->float_param, 333.0f);
    EXPECT_EQ(ulk->string_param, "mutexed_value");
    ulk->string_param = "";
    EXPECT_EQ(params_lck.unsafe_access().string_param, "");
    *ulk = {true, 43, 53.0f, "unlocked_value"};
    EXPECT_FALSE(params_lck.mutex().try_lock());
  }
  EXPECT_TRUE(params_lck.mutex().try_lock());
  params_lck.mutex().unlock();
  EXPECT_EQ(params_lck.unsafe_access().bool_param, true);
  EXPECT_EQ(params_lck.unsafe_access().int_param, 43);
  EXPECT_EQ(params_lck.unsafe_access().float_param, 53.0f);
  EXPECT_EQ(params_lck.unsafe_access().string_param, "unlocked_value");
}

//}
