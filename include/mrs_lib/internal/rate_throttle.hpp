#ifndef MRS_LIB_INTERNAL_RATE_THROTTLE_HPP_
#define MRS_LIB_INTERNAL_RATE_THROTTLE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace mrs_lib::internal
{

  /**
   * @brief Decides which messages of a stream to pass so that the output rate does not exceed a cap.
   *
   * Deadline (GCRA-like) scheduling: the next allowed time advances by one period per passed message,
   * so a faster, jittery input is decimated to the cap on average instead of undershooting it.
   * A message up to period/4 early is passed, a late message keeps at most period/2 of catch-up
   * credit, so there are no bursts after a pause (passed messages are at least period/4 apart).
   * An input slower than the cap is passed as is. Not thread-safe.
   */
  class RateThrottle
  {
  public:
    /**
     * @param rate maximum output rate [Hz], <= 0 disables throttling
     */
    explicit RateThrottle(const double rate) : RateThrottle(std::chrono::nanoseconds(rate > 0 ? std::llround(std::min(1e9 / rate, double(max_period_ns_))) : 0))
    {
    }

    /**
     * @param period minimum output period, <= 0 disables throttling
     */
    explicit RateThrottle(const std::chrono::nanoseconds period)
        : period_(rclcpp::Duration::from_nanoseconds(std::clamp<int64_t>(period.count(), 0, max_period_ns_))),
          jitter_tolerance_(rclcpp::Duration::from_nanoseconds(period_.nanoseconds() / 4)),
          max_catch_up_(rclcpp::Duration::from_nanoseconds(period_.nanoseconds() / 2)),
          clock_jump_threshold_(rclcpp::Duration::from_nanoseconds((3 * period_.nanoseconds()) / 2))
    {
    }

    /**
     * @brief decide whether a message arriving at \p now should be passed (and register it if so)
     *
     * @param now current time, all calls have to use the same clock type
     *
     * @return true if the message should be passed
     *
     * @throws std::runtime_error if \p now has a different clock type than the previous calls
     */
    bool accept(const rclcpp::Time& now)
    {
      if (period_.nanoseconds() <= 0)
      {
        return true;
      }

      // first message, or the clock jumped backwards (e.g., sim time reset)
      if (!next_deadline_ || now < *next_deadline_ - clock_jump_threshold_)
      {
        next_deadline_ = now;
      }

      if (now < *next_deadline_ - jitter_tolerance_)
      {
        return false;
      }

      next_deadline_ = std::max(*next_deadline_ + period_, now + max_catch_up_);

      return true;
    }

  private:
    // keeps the deadline arithmetic far from int64 overflow (~73 years)
    static constexpr int64_t max_period_ns_ = std::numeric_limits<int64_t>::max() / 4;

    rclcpp::Duration period_;

    rclcpp::Duration jitter_tolerance_;     // a message this much before the deadline still passes
    rclcpp::Duration max_catch_up_;         // the next deadline is at least this far after a passed message, limits catch-up after a pause
    rclcpp::Duration clock_jump_threshold_; // a message this much before the deadline means the clock jumped backwards

    std::optional<rclcpp::Time> next_deadline_;
  };

} // namespace mrs_lib::internal

#endif // MRS_LIB_INTERNAL_RATE_THROTTLE_HPP_
