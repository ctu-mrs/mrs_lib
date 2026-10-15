#include "mrs_lib/internal/rate_throttle.hpp"

#include <gtest/gtest.h>

#include <rclcpp/time.hpp>

#include <chrono>
#include <cstdint>
#include <random>
#include <vector>

namespace
{

  constexpr int64_t NS = 1000000000;

  // timestamps [ns] of a stream at the given rate, with a uniform jitter of +-jitter_ns
  std::vector<int64_t> makeStream(const double rate, const double duration, const int64_t jitter_ns = 0, const int64_t start_ns = 0)
  {
    std::mt19937_64 gen(42);

    std::vector<int64_t> stamps;

    const int n = int(rate * duration);

    for (int i = 0; i < n; i++)
    {
      int64_t jitter = jitter_ns > 0 ? int64_t(gen() % uint64_t(2 * jitter_ns + 1)) - jitter_ns : 0;
      stamps.push_back(start_ns + int64_t(double(i) * NS / rate) + jitter);
    }

    return stamps;
  }

  std::vector<int64_t> runThrottle(mrs_lib::internal::RateThrottle& throttle, const std::vector<int64_t>& stamps)
  {
    std::vector<int64_t> out;

    for (const auto t : stamps)
    {
      if (throttle.accept(rclcpp::Time(t)))
      {
        out.push_back(t);
      }
    }

    return out;
  }

  double outputRate(const std::vector<int64_t>& out)
  {
    return double(out.size() - 1) / (double(out.back() - out.front()) / NS);
  }

  int64_t minGap(const std::vector<int64_t>& out)
  {
    int64_t gap = INT64_MAX;

    for (size_t i = 1; i < out.size(); i++)
    {
      gap = std::min(gap, out[i] - out[i - 1]);
    }

    return gap;
  }

} // namespace

/* TEST(RateThrottle, first_message) //{ */

TEST(RateThrottle, first_message)
{

  mrs_lib::internal::RateThrottle throttle(1.0);

  EXPECT_TRUE(throttle.accept(rclcpp::Time(123 * NS)));
  EXPECT_FALSE(throttle.accept(rclcpp::Time(123 * NS + NS / 2)));
  EXPECT_TRUE(throttle.accept(rclcpp::Time(124 * NS)));
}

//}

/* TEST(RateThrottle, disabled) //{ */

TEST(RateThrottle, disabled)
{

  mrs_lib::internal::RateThrottle throttle(0.0);

  const auto stamps = makeStream(1000.0, 1.0);

  EXPECT_EQ(runThrottle(throttle, stamps).size(), stamps.size());
}

//}

/* TEST(RateThrottle, period_constructor) //{ */

TEST(RateThrottle, period_constructor)
{

  // the period form (used by mrs_lib::Publisher) behaves like the rate form
  mrs_lib::internal::RateThrottle by_rate(50.0);
  mrs_lib::internal::RateThrottle by_period(std::chrono::milliseconds(20));
  mrs_lib::internal::RateThrottle disabled(std::chrono::nanoseconds(0));

  const auto stamps = makeStream(99.9, 5.0, 1500000);

  EXPECT_EQ(runThrottle(by_period, stamps), runThrottle(by_rate, stamps));
  EXPECT_EQ(runThrottle(disabled, stamps).size(), stamps.size());
}

//}

/* TEST(RateThrottle, huge_period) //{ */

TEST(RateThrottle, huge_period)
{

  // "publish once" style periods are clamped instead of overflowing the deadline arithmetic
  mrs_lib::internal::RateThrottle by_period(std::chrono::nanoseconds::max());
  mrs_lib::internal::RateThrottle by_rate(1e-12);

  const int64_t start_ns = 1'800'000'000'000'000'000; // ~2027 in ns since epoch
  const int64_t hour_ns = 3'600'000'000'000;

  for (auto* throttle : {&by_period, &by_rate})
  {
    EXPECT_TRUE(throttle->accept(rclcpp::Time(start_ns)));
    EXPECT_FALSE(throttle->accept(rclcpp::Time(start_ns + hour_ns)));
  }
}

//}

/* TEST(RateThrottle, jittery_fast_input) //{ */

TEST(RateThrottle, jittery_fast_input)
{

  // 99.9 Hz with +-1.5 ms jitter capped to 50 Hz (the naive "dt since last" throttle gives ~38 Hz)
  mrs_lib::internal::RateThrottle throttle(50.0);

  const auto out = runThrottle(throttle, makeStream(99.9, 20.0, 1500000));

  EXPECT_NEAR(outputRate(out), 50.0, 0.1);
  EXPECT_GE(minGap(out), NS / 50 / 4);
}

//}

/* TEST(RateThrottle, input_just_above_cap) //{ */

TEST(RateThrottle, input_just_above_cap)
{

  // 52 Hz capped to 50 Hz (the naive "dt since last" throttle gives ~26-30 Hz)
  {
    mrs_lib::internal::RateThrottle throttle(50.0);

    const auto out = runThrottle(throttle, makeStream(52.0, 20.0));

    EXPECT_GE(outputRate(out), 49.0);
    EXPECT_LE(outputRate(out), 50.1);
  }

  {
    mrs_lib::internal::RateThrottle throttle(50.0);

    const auto out = runThrottle(throttle, makeStream(52.0, 20.0, 2000000));

    EXPECT_GE(outputRate(out), 49.0);
    EXPECT_LE(outputRate(out), 50.1);
  }
}

//}

/* TEST(RateThrottle, slow_input_passes) //{ */

TEST(RateThrottle, slow_input_passes)
{

  {
    mrs_lib::internal::RateThrottle throttle(50.0);

    const auto stamps = makeStream(30.0, 10.0);

    EXPECT_EQ(runThrottle(throttle, stamps).size(), stamps.size());
  }

  {
    mrs_lib::internal::RateThrottle throttle(50.0);

    const auto stamps = makeStream(45.0, 10.0, 1000000);

    EXPECT_EQ(runThrottle(throttle, stamps).size(), stamps.size());
  }
}

//}

/* TEST(RateThrottle, no_burst_after_pause) //{ */

TEST(RateThrottle, no_burst_after_pause)
{

  mrs_lib::internal::RateThrottle throttle(50.0);

  // 1 kHz input for 1 s, a 3 s pause, then 1 kHz again
  auto stamps = makeStream(1000.0, 1.0);

  const int64_t resume_ns = 4 * NS;

  for (const auto t : makeStream(1000.0, 1.0, 0, resume_ns))
  {
    stamps.push_back(t);
  }

  const auto out = runThrottle(throttle, stamps);

  EXPECT_GE(minGap(out), NS / 50 / 4);

  // at most one extra message (the first one) within the first 10 periods after the pause
  int after_pause = 0;

  for (const auto t : out)
  {
    if (t >= resume_ns && t < resume_ns + NS / 5)
    {
      after_pause++;
    }
  }

  EXPECT_LE(after_pause, 11);
  EXPECT_GE(after_pause, 10);
}

//}

/* TEST(RateThrottle, backwards_clock_jump) //{ */

TEST(RateThrottle, backwards_clock_jump)
{

  mrs_lib::internal::RateThrottle throttle(50.0);

  // 100 Hz input for 10 s, then the clock is reset back to 0 (e.g., sim time restart)
  runThrottle(throttle, makeStream(100.0, 10.0, 0, 0));

  const auto out = runThrottle(throttle, makeStream(100.0, 2.0, 0, 0));

  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out.front(), 0);
  EXPECT_NEAR(outputRate(out), 50.0, 0.5);
}

//}
