#include "mrs_lib/coro/event.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <latch>
#include <thread>

#include "mrs_lib/coro/runners.hpp"
#include "mrs_lib/coro/task.hpp"
#include "mrs_lib/utility/scope_cleanup.hpp"

namespace
{
  struct DataOut
  {
    std::atomic<bool> finished = false;
    std::atomic<bool> destroyed = false;
  };

  mrs_lib::Task<> wait_for_event(mrs_lib::coro::EventAwaitable event_awaitable, DataOut& out)
  {
    mrs_lib::ScopeCleanup set_destroyed_clenup([&] { out.destroyed = true; });
    co_await std::move(event_awaitable).wait();
    out.finished = true;
  }

  mrs_lib::Task<> wait_for_event_store_waker(DataOut& out, std::function<void()>& waker)
  {
    auto [event, awaitable] = mrs_lib::coro::make_event();
    using StopTokenBehavior = mrs_lib::coro::internal::LowLevelEventAwaitable::StopTokenBehavior;
    mrs_lib::ScopeCleanup set_destroyed_clenup([&] { out.destroyed = true; });
    auto low_level_awaitable = mrs_lib::coro::internal::get_low_level_event_awaitable(std::move(awaitable));
    co_await std::move(low_level_awaitable).get_awaitable(StopTokenBehavior::respect, [&] {
      waker = [shared_waker = std::make_shared<mrs_lib::coro::Event>(std::move(event))] { shared_waker->try_trigger(); };
    });
    out.finished = true;
  }

  TEST(MrsLibCoroEvent, EventTriggerAfterSuspend)
  {
    DataOut data_out{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    mrs_lib::coro::internal::start_task(wait_for_event, std::move(awaitable), std::ref(data_out));

    EXPECT_FALSE(data_out.finished);
    EXPECT_FALSE(data_out.destroyed);

    ASSERT_TRUE(event.try_trigger());

    EXPECT_TRUE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, EventTriggerBeforeSuspend)
  {
    DataOut data_out{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    ASSERT_TRUE(event.try_trigger());

    mrs_lib::coro::internal::start_task(wait_for_event, std::move(awaitable), std::ref(data_out));
    EXPECT_TRUE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, EventCancelAfterSuspend)
  {
    DataOut data_out{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    mrs_lib::coro::internal::start_task(wait_for_event, std::move(awaitable), std::ref(data_out));

    EXPECT_FALSE(data_out.finished);
    EXPECT_FALSE(data_out.destroyed);

    ASSERT_TRUE(event.try_cancel());

    EXPECT_FALSE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, EventCancelBeforeSuspend)
  {
    DataOut data_out{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    ASSERT_TRUE(event.try_cancel());

    mrs_lib::coro::internal::start_task(wait_for_event, std::move(awaitable), std::ref(data_out));
    EXPECT_FALSE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, EventCancelOnDestroy)
  {
    DataOut data_out{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    mrs_lib::coro::internal::start_task(wait_for_event, std::move(awaitable), std::ref(data_out));

    EXPECT_FALSE(data_out.finished);
    EXPECT_FALSE(data_out.destroyed);

    {
      mrs_lib::coro::Event inner_event = std::move(event);
      EXPECT_FALSE(data_out.finished);
      EXPECT_FALSE(data_out.destroyed);
    }

    EXPECT_FALSE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, SecondTriggerFails)
  {
    using mrs_lib::coro::EventError;

    auto [event, awaitable] = mrs_lib::coro::make_event();

    // First should trigger
    ASSERT_TRUE(event.try_trigger());
    // Second should fail
    EXPECT_FALSE(event.try_cancel());
  }

  TEST(MrsLibCoroEvent, SecondCancelFails)
  {
    using mrs_lib::coro::EventError;

    auto [event, awaitable] = mrs_lib::coro::make_event();

    // First should cancel
    ASSERT_TRUE(event.try_cancel());
    // Second should fail
    EXPECT_FALSE(event.try_cancel());
  }

  TEST(MrsLibCoroEvent, SecondAwaitThrows)
  {
    using mrs_lib::coro::EventError;

    DataOut data_out{};
    auto [event, awaitable] = mrs_lib::coro::make_event();
    ASSERT_TRUE(event.try_trigger());

    mrs_lib::coro::internal::start_task(
        [](mrs_lib::coro::EventAwaitable event_awaitable, DataOut& out) -> mrs_lib::Task<> {
          mrs_lib::ScopeCleanup set_destroyed_clenup([&] { out.destroyed = true; });
          // First should succeed
          EXPECT_NO_THROW(co_await std::move(event_awaitable).wait());
          // Second should throw
          EXPECT_THROW(co_await std::move(event_awaitable).wait(), mrs_lib::coro::EventError);
          out.finished = true;
        },
        std::move(awaitable), std::ref(data_out));

    EXPECT_TRUE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, StopTokenCancelAfterSuspend)
  {
    DataOut data_out{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    std::stop_source stop_source{};
    mrs_lib::coro::internal::start_task(stop_source.get_token(), wait_for_event, std::move(awaitable), std::ref(data_out));

    EXPECT_FALSE(data_out.finished);
    EXPECT_FALSE(data_out.destroyed);

    stop_source.request_stop();

    EXPECT_FALSE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, WakerCanResumeImmediately)
  {
    DataOut data_out{};

    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<> {
      auto [event, awaitable] = mrs_lib::coro::make_event();
      using StopTokenBehavior = mrs_lib::coro::internal::LowLevelEventAwaitable::StopTokenBehavior;
      mrs_lib::ScopeCleanup set_destroyed_clenup([&] { data_out.destroyed = true; });
      auto low_level_awaitable = mrs_lib::coro::internal::get_low_level_event_awaitable(std::move(awaitable));
      co_await std::move(low_level_awaitable)
          .get_awaitable(StopTokenBehavior::respect,
                         [shared_event = std::make_shared<mrs_lib::coro::Event>(std::move(event))]() { shared_event->try_trigger(); });
      data_out.finished = true;
    });

    EXPECT_TRUE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, WakerCanResumeLater)
  {
    DataOut data_out{};

    std::function<void()> waker;

    mrs_lib::coro::internal::start_task(wait_for_event_store_waker, std::ref(data_out), std::ref(waker));

    EXPECT_FALSE(data_out.finished);
    EXPECT_FALSE(data_out.destroyed);

    EXPECT_TRUE(static_cast<bool>(waker));
    waker();

    EXPECT_TRUE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, DroppedWakerCancels)
  {
    DataOut data_out{};

    std::function<void()> waker;

    mrs_lib::coro::internal::start_task(wait_for_event_store_waker, std::ref(data_out), std::ref(waker));

    EXPECT_FALSE(data_out.finished);
    EXPECT_FALSE(data_out.destroyed);

    EXPECT_TRUE(static_cast<bool>(waker));
    waker = {};
    EXPECT_FALSE(static_cast<bool>(waker));

    EXPECT_FALSE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

  TEST(MrsLibCoroEvent, CancelDuringEventAwaitableCallback)
  {
    std::latch callback_entered(2);
    std::latch allow_callback_to_return(2);

    std::atomic<bool> started = false;
    DataOut data_out{};

    std::atomic<bool> entered_callback = false;


    auto task = [&](mrs_lib::coro::EventAwaitable awaitable) -> mrs_lib::coro::Task<void> {
      started = true;
      mrs_lib::ScopeCleanup set_destroyed_clenup([&] { data_out.destroyed = true; });

      auto low_level_awaitable = mrs_lib::coro::internal::get_low_level_event_awaitable(std::move(awaitable));

      co_await std::move(low_level_awaitable).get_awaitable(mrs_lib::coro::internal::LowLevelEventAwaitable::StopTokenBehavior::respect, [&] {
        entered_callback = true;
        callback_entered.arrive_and_wait();
        // Other thread requests cancellation in this time...
        allow_callback_to_return.arrive_and_wait();
      });

      data_out.finished = true;
    };

    auto [event, awaitable] = mrs_lib::coro::make_event();

    // Run the task on a separate thread.
    std::jthread runner([](auto task, auto awaitable) { mrs_lib::coro::internal::start_task(std::move(task), std::move(awaitable)); }, std::move(task),
                        std::move(awaitable));

    // Wait for the task to enter the callback
    callback_entered.arrive_and_wait();

    ASSERT_TRUE(entered_callback);

    {
      // Move event here to destroy it before resuming the callback.
      mrs_lib::coro::Event inner_event = std::move(event);
      bool cancel_result = inner_event.try_cancel();
      ASSERT_TRUE(cancel_result);
    }

    allow_callback_to_return.arrive_and_wait();

    runner.join();

    EXPECT_FALSE(data_out.finished);
    EXPECT_TRUE(data_out.destroyed);
  }

} // namespace
