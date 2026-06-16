#include "mrs_lib/coro/cancellation.hpp"

#include <gtest/gtest.h>

#include "mrs_lib/coro/runners.hpp"
#include "mrs_lib/coro/task.hpp"


namespace
{

  mrs_lib::Task<> store_own_stop_token(std::stop_token& out_token, bool& finished)
  {
    out_token = co_await mrs_lib::coro::get_task_stop_token();
    finished = true;
  }

  TEST(MrsLibCoroCancellation, StopTokenPassed)
  {
    bool finished = false;
    std::stop_token out_token{};
    std::stop_source stop_source{};

    mrs_lib::coro::internal::start_task(stop_source.get_token(), &store_own_stop_token, std::ref(out_token), std::ref(finished));

    ASSERT_TRUE(finished);
    EXPECT_TRUE(out_token.stop_possible());
    EXPECT_EQ(out_token, stop_source.get_token());
  }

  TEST(MrsLibCoroCancellation, StopTokenNotPassed)
  {
    bool finished = false;
    std::stop_token out_token{};

    mrs_lib::coro::internal::start_task(&store_own_stop_token, std::ref(out_token), std::ref(finished));

    ASSERT_TRUE(finished);
    EXPECT_FALSE(out_token.stop_possible());
    EXPECT_EQ(out_token, std::stop_token{});
  }

} // namespace
