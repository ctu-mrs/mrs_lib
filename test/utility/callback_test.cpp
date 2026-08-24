#include "mrs_lib/utility/callback.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <string_view>
#include <type_traits>
#include <utility>

#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

#include "mrs_lib/coro/event.hpp"
#include "mrs_lib/coro/runners.hpp"
#include "mrs_lib/coro/task.hpp"
#include "mrs_lib/utility/scope_cleanup.hpp"


namespace
{

  //////////////////////////////////////////////////////////////////////////////
  //  CoroCallbackExample                                                     //
  //////////////////////////////////////////////////////////////////////////////

  // DOCS: BEGIN EXAMPLE P1
  mrs_lib::Task<int> add(int a, int b)
  {
    co_return a + b;
  }

  class MyNode : public rclcpp::Node
  {
  public:
    using rclcpp::Node::Node;

    mrs_lib::CoroCallback<int(int)> create_callback()
    {
      using mrs_lib::coro_callback_tags::Reentrant;
      // Binding to methods of nodes. A callback like this would be typically
      // passed to mrs_lib ros wrappers (timer, subscriber, ...).
      return mrs_lib::CoroCallback(Reentrant{}, &MyNode::calculate, this);
    }

    mrs_lib::Task<int> calculate(int x)
    {
      co_return x + val_;
    }

  private:
    int val_ = 10;
  };
  // DOCS: END EXAMPLE P1


  TEST(CoroCallbackExample, Example)
  {
    rclcpp::init(0, nullptr);

    // DOCS: BEGIN EXAMPLE P2
    using mrs_lib::CoroCallback;
    using mrs_lib::coro_callback_tags::CancelNew;
    using mrs_lib::coro_callback_tags::CancelNewDefault;
    using mrs_lib::coro_callback_tags::Reentrant;

    // Reentrant callbacks
    // No argument bound
    CoroCallback<int(int, int)> callback1 = CoroCallback(Reentrant{}, &add);
    // First argument bound
    CoroCallback<int(int)> callback2 = CoroCallback(Reentrant{}, &add, 1);

    // Cancel new callback
    // This callback will return default constructed int (0) if cancelled by the policy
    CoroCallback<int(int)> callback3 = CoroCallback(CancelNewDefault{}, &add, 1);
    // This callback will return 42 if cancelled by the policy
    CoroCallback<int(int)> callback4 = CoroCallback(CancelNew([] { return 42; }), &add, 1);
    // DOCS: END EXAMPLE P2

    auto node = std::make_shared<MyNode>("my_node");
    CoroCallback<int(int)> callback5 = node->create_callback();

    {
      int res = 0;
      mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback1(21, 21); });
      EXPECT_EQ(res, 42);
    }

    {
      int res = 0;
      mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback2(41); });
      EXPECT_EQ(res, 42);
    }

    {
      int res = 0;
      mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback3(41); });
      EXPECT_EQ(res, 42);
    }

    {
      int res = 0;
      mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback4(41); });
      EXPECT_EQ(res, 42);
    }

    {
      int res = 0;
      mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback5(32); });
      EXPECT_EQ(res, 42);
    }

    rclcpp::shutdown();
  }


  //////////////////////////////////////////////////////////////////////////////
  //  Test utilities                                                          //
  //////////////////////////////////////////////////////////////////////////////

  using mrs_lib::coro_callback_tags::CancelNew;
  using mrs_lib::coro_callback_tags::CancelNewDefault;
  using mrs_lib::coro_callback_tags::Reentrant;

  static_assert(std::same_as<mrs_lib::internal::CallbackSignatureT<void, mrs_lib::meta::TypeList<>, mrs_lib::meta::TypeList<>>, void()>);
  static_assert(std::same_as<mrs_lib::internal::CallbackSignatureT<void, mrs_lib::meta::TypeList<int>, mrs_lib::meta::TypeList<>>, void(int)>);
  static_assert(std::same_as<mrs_lib::internal::CallbackSignatureT<void, mrs_lib::meta::TypeList<int, double, bool>, mrs_lib::meta::TypeList<const int&>>,
                             void(double, bool)>);


  mrs_lib::Task<int> co_get_42()
  {
    co_return 42;
  }

  mrs_lib::Task<int> co_add(int a, int b) noexcept
  {
    co_return a + b;
  }

  mrs_lib::Task<int> co_add_ref(const int& a, const int& b)
  {
    co_return a + b;
  }

  class TestClassMutable
  {
  public:
    mrs_lib::Task<int> co_get_42()
    {
      ++calls_;
      co_return 42;
    }

    mrs_lib::Task<int> co_add(int a, int b) noexcept
    {
      ++calls_;
      co_return a + b;
    }

    mrs_lib::Task<int> co_add_ref(const int& a, const int& b)
    {
      ++calls_;
      co_return a + b;
    }

    int get_calls() const
    {
      return calls_;
    }

  private:
    int calls_ = 0;
  };

  class TestClassConst
  {
  public:
    mrs_lib::Task<int> co_get_42() const
    {
      ++calls_;
      co_return 42;
    }

    mrs_lib::Task<int> co_add(int a, int b) const noexcept
    {
      ++calls_;
      co_return a + b;
    }

    mrs_lib::Task<int> co_add_ref(const int& a, const int& b) const
    {
      ++calls_;
      co_return a + b;
    }

    int get_calls() const
    {
      return calls_;
    }

  private:
    mutable int calls_ = 0;
  };

  class OutState
  {
  public:
    bool started = false;
    bool finished = false;
    bool destroyed = false;

    bool wrapper_finished = false;

    void check_started(std::string_view msg = "")
    {
      EXPECT_TRUE(this->started) << msg;
      EXPECT_FALSE(this->finished) << msg;
      EXPECT_FALSE(this->destroyed) << msg;
      EXPECT_FALSE(this->wrapper_finished) << msg;
    }

    void check_finished(std::string_view msg = "")
    {
      EXPECT_TRUE(this->started) << msg;
      EXPECT_TRUE(this->finished) << msg;
      EXPECT_TRUE(this->destroyed) << msg;
      EXPECT_TRUE(this->wrapper_finished) << msg;
    }

    void check_skipped(std::string_view msg = "")
    {
      EXPECT_FALSE(this->started) << msg;
      EXPECT_FALSE(this->finished) << msg;
      EXPECT_FALSE(this->destroyed) << msg;
      EXPECT_TRUE(this->wrapper_finished) << msg;
    }
  };

  mrs_lib::Task<> co_wait_for_event(OutState& state, mrs_lib::coro::EventAwaitable awaitable)
  {
    state.started = true;
    mrs_lib::ScopeCleanup cleanup_set_destroyed([&] { state.destroyed = true; });
    co_await std::move(awaitable).wait();
    state.finished = true;
  }

  // Using unique ptr to test move only return type
  mrs_lib::Task<std::unique_ptr<int>> co_wait_for_event_ret(OutState& state, mrs_lib::coro::EventAwaitable awaitable)
  {
    state.started = true;
    mrs_lib::ScopeCleanup cleanup_set_destroyed([&] { state.destroyed = true; });
    co_await std::move(awaitable).wait();
    state.finished = true;
    co_return std::make_unique<int>(42);
  }

  void start_callback_with_state(const mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>& callback, OutState& state,
                                 mrs_lib::coro::EventAwaitable&& awaitable)
  {
    mrs_lib::coro::internal::start_task([callback, &state, awaitable = std::move(awaitable)]() mutable -> mrs_lib::Task<> {
      co_await callback(state, std::move(awaitable));
      state.wrapper_finished = true;
    });
  }

  template <typename T>
  void start_callback_with_state(const mrs_lib::CoroCallback<T(OutState&, mrs_lib::coro::EventAwaitable)>& callback,
                                 std::type_identity_t<std::function<void(T)>> consumer, OutState& state, mrs_lib::coro::EventAwaitable&& awaitable)
  {
    mrs_lib::coro::internal::start_task([callback, consumer = std::move(consumer), &state, awaitable = std::move(awaitable)]() mutable -> mrs_lib::Task<> {
      consumer(co_await callback(state, std::move(awaitable)));
      state.wrapper_finished = true;
    });
  }

  //////////////////////////////////////////////////////////////////////////////
  //  CoroCallbackCtor                                                        //
  //////////////////////////////////////////////////////////////////////////////

  // Free functions

  TEST(CoroCallbackCtor, NoParams)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_get_42);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
  }

  TEST(CoroCallbackCtor, NoBind)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_add);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(int, int)>, decltype(callback)>);

    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21, 21); });
    EXPECT_EQ(res, 42);
  }

  TEST(CoroCallbackCtor, FullBind)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_add, 21, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
  }

  TEST(CoroCallbackCtor, PartialBind)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_add, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(int)>, decltype(callback)>);

    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21); });
    EXPECT_EQ(res, 42);
  }

  TEST(CoroCallbackCtor, NoBindRef)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_add_ref);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(const int&, const int&)>, decltype(callback)>);

    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21, 21); });
    EXPECT_EQ(res, 42);
  }

  TEST(CoroCallbackCtor, FullBindRef)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_add_ref, 21, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
  }

  TEST(CoroCallbackCtor, PartialBindRef)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_add_ref, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(const int&)>, decltype(callback)>);

    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21); });
    EXPECT_EQ(res, 42);
  }

  // Methods with object by reference

  TEST(CoroCallbackCtor, MethodRefNoParams)
  {
    TestClassMutable obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassMutable::co_get_42, std::ref(obj));
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 1);
  }

  TEST(CoroCallbackCtor, MethodRefNoBind)
  {
    TestClassMutable obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassMutable::co_add, std::ref(obj));
    static_assert(std::same_as<mrs_lib::CoroCallback<int(int, int)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21, 21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 1);
  }

  TEST(CoroCallbackCtor, MethodRefFullBind)
  {
    TestClassMutable obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassMutable::co_add, std::ref(obj), 21, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 1);
  }

  TEST(CoroCallbackCtor, MethodRefPartialBind)
  {
    TestClassMutable obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassMutable::co_add, std::ref(obj), 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(int)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 1);
  }

  TEST(CoroCallbackCtor, MethodRefNoBindRef)
  {
    TestClassMutable obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassMutable::co_add_ref, std::ref(obj));
    static_assert(std::same_as<mrs_lib::CoroCallback<int(const int&, const int&)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21, 21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 1);
  }

  TEST(CoroCallbackCtor, MethodRefFullBindRef)
  {
    TestClassMutable obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassMutable::co_add_ref, std::ref(obj), 21, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 1);
  }

  TEST(CoroCallbackCtor, MethodRefPartialBindRef)
  {
    TestClassMutable obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassMutable::co_add_ref, std::ref(obj), 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(const int&)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 1);
  }

  // Methods with object by value

  TEST(CoroCallbackCtor, MethodValNoParams)
  {
    TestClassConst obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassConst::co_get_42, obj);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 0);
  }

  TEST(CoroCallbackCtor, MethodValNoBind)
  {
    TestClassConst obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassConst::co_add, obj);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(int, int)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21, 21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 0);
  }

  TEST(CoroCallbackCtor, MethodValFullBind)
  {
    TestClassConst obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassConst::co_add, obj, 21, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 0);
  }

  TEST(CoroCallbackCtor, MethodValPartialBind)
  {
    TestClassConst obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassConst::co_add, obj, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(int)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 0);
  }


  TEST(CoroCallbackCtor, MethodValNoBindRef)
  {
    TestClassConst obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassConst::co_add_ref, obj);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(const int&, const int&)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21, 21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 0);
  }

  TEST(CoroCallbackCtor, MethodValFullBindRef)
  {
    TestClassConst obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassConst::co_add_ref, obj, 21, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int()>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 0);
  }

  TEST(CoroCallbackCtor, MethodValPartialBindRef)
  {
    TestClassConst obj{};
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &TestClassConst::co_add_ref, obj, 21);
    static_assert(std::same_as<mrs_lib::CoroCallback<int(const int&)>, decltype(callback)>);

    EXPECT_EQ(obj.get_calls(), 0);
    int res = 0;
    mrs_lib::coro::internal::start_task([&]() -> mrs_lib::Task<void> { res = co_await callback(21); });
    EXPECT_EQ(res, 42);
    EXPECT_EQ(obj.get_calls(), 0);
  }

  //////////////////////////////////////////////////////////////////////////////
  //  CoroCallbackReentrant                                                   //
  //////////////////////////////////////////////////////////////////////////////

  TEST(CoroCallbackReentrant, SingleCall)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &co_wait_for_event);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);


    OutState state{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    start_callback_with_state(callback, state, std::move(awaitable));
    state.check_started();

    event.try_trigger();
    state.check_finished();
  }

  TEST(CoroCallbackReentrant, Sequenced)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &co_wait_for_event);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);

    for (size_t i = 0; i < 2; ++i)
    {
      OutState state{};
      auto [event, awaitable] = mrs_lib::coro::make_event();

      start_callback_with_state(callback, state, std::move(awaitable));
      state.check_started();

      event.try_trigger();
      state.check_finished();
    }
  }

  TEST(CoroCallbackReentrant, Concurent)
  {
    auto callback = mrs_lib::CoroCallback(Reentrant{}, &co_wait_for_event);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);

    OutState state1{};
    auto [event1, awaitable1] = mrs_lib::coro::make_event();

    OutState state2{};
    auto [event2, awaitable2] = mrs_lib::coro::make_event();

    // Start first callback
    start_callback_with_state(callback, state1, std::move(awaitable1));
    state1.check_started("callback 1");
    // Start second callback
    // Since it is reentrant, it should behave the same as the first
    start_callback_with_state(callback, state2, std::move(awaitable2));
    state2.check_started("callback 2");

    // Resume first callback
    event1.try_trigger();
    state1.check_finished("callback 1");

    // Resume second callback
    event2.try_trigger();
    state2.check_finished("callback 2");
  }

  //////////////////////////////////////////////////////////////////////////////
  //  CoroCallbackCancelNew                                                   //
  //////////////////////////////////////////////////////////////////////////////

  TEST(CoroCallbackCancelNew, SingleCall)
  {
    auto callback = mrs_lib::CoroCallback(CancelNewDefault{}, &co_wait_for_event);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);


    OutState state{};
    auto [event, awaitable] = mrs_lib::coro::make_event();

    start_callback_with_state(callback, state, std::move(awaitable));
    state.check_started();

    event.try_trigger();
    state.check_finished();
  }

  TEST(CoroCallbackCancelNew, Sequenced)
  {
    auto callback = mrs_lib::CoroCallback(CancelNewDefault{}, &co_wait_for_event);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);

    for (size_t i = 0; i < 2; ++i)
    {
      OutState state{};
      auto [event, awaitable] = mrs_lib::coro::make_event();

      start_callback_with_state(callback, state, std::move(awaitable));
      state.check_started();

      event.try_trigger();
      state.check_finished();
    }
  }

  TEST(CoroCallbackCancelNew, Concurent)
  {
    auto callback = mrs_lib::CoroCallback(CancelNewDefault{}, &co_wait_for_event);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);

    OutState state1{};
    auto [event1, awaitable1] = mrs_lib::coro::make_event();

    OutState state2{};
    auto [event2, awaitable2] = mrs_lib::coro::make_event();

    // Start first callback
    start_callback_with_state(callback, state1, std::move(awaitable1));
    state1.check_started("callback 1");
    // Start second callback
    // Since it is `cancel new`, it should not start at all
    start_callback_with_state(callback, state2, std::move(awaitable2));
    state2.check_skipped("callback 2");

    // Resume first callback
    event1.try_trigger();
    state1.check_finished("callback 1");

    // Resume second callback - this should do nothing
    event2.try_trigger();
    state2.check_skipped("callback 2");
  }

  TEST(CoroCallbackCancelNew, ConcurentNonDefaultVoidRet)
  {
    auto callback = mrs_lib::CoroCallback(CancelNew<>{}, &co_wait_for_event);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);

    OutState state1{};
    auto [event1, awaitable1] = mrs_lib::coro::make_event();

    OutState state2{};
    auto [event2, awaitable2] = mrs_lib::coro::make_event();

    // Start first callback
    start_callback_with_state(callback, state1, std::move(awaitable1));
    state1.check_started("callback 1");
    // Start second callback
    // Since it is `cancel new`, it should not start at all
    start_callback_with_state(callback, state2, std::move(awaitable2));
    state2.check_skipped("callback 2");

    // Resume first callback
    event1.try_trigger();
    state1.check_finished("callback 1");

    // Resume second callback - this should do nothing
    event2.try_trigger();
    state2.check_skipped("callback 2");
  }

  TEST(CoroCallbackCancelNew, ConcurentDefaultUniquePtrRet)
  {
    auto callback = mrs_lib::CoroCallback(CancelNewDefault{}, &co_wait_for_event_ret);
    static_assert(std::same_as<mrs_lib::CoroCallback<std::unique_ptr<int>(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);

    OutState state1{};
    auto [event1, awaitable1] = mrs_lib::coro::make_event();

    OutState state2{};
    auto [event2, awaitable2] = mrs_lib::coro::make_event();

    // Start first callback
    start_callback_with_state(
        callback,
        [](std::unique_ptr<int> ptr) {
          ASSERT_NE(ptr, nullptr);
          EXPECT_EQ(*ptr, 42);
        },
        state1, std::move(awaitable1));
    state1.check_started("callback 1");
    // Start second callback
    // Since it is `cancel new`, it should not start at all
    start_callback_with_state(callback, [](std::unique_ptr<int> ptr) { EXPECT_EQ(ptr, nullptr); }, state2, std::move(awaitable2));
    state2.check_skipped("callback 2");

    // Resume first callback
    event1.try_trigger();
    state1.check_finished("callback 1");

    // Resume second callback - this should do nothing
    event2.try_trigger();
    state2.check_skipped("callback 2");
  }

  TEST(CoroCallbackCancelNew, ConcurentNonDefaultUniquePtrRet)
  {
    auto callback = mrs_lib::CoroCallback(CancelNew([] { return std::make_unique<int>(69); }), &co_wait_for_event_ret);
    static_assert(std::same_as<mrs_lib::CoroCallback<std::unique_ptr<int>(OutState&, mrs_lib::coro::EventAwaitable)>, decltype(callback)>);

    OutState state1{};
    auto [event1, awaitable1] = mrs_lib::coro::make_event();

    OutState state2{};
    auto [event2, awaitable2] = mrs_lib::coro::make_event();

    // Start first callback
    start_callback_with_state(
        callback,
        [](std::unique_ptr<int> ptr) {
          ASSERT_NE(ptr, nullptr);
          EXPECT_EQ(*ptr, 42);
        },
        state1, std::move(awaitable1));
    state1.check_started("callback 1");
    // Start second callback
    // Since it is `cancel new`, it should not start at all
    start_callback_with_state(
        callback,
        [](std::unique_ptr<int> ptr) {
          ASSERT_NE(ptr, nullptr);
          EXPECT_EQ(*ptr, 69);
        },
        state2, std::move(awaitable2));
    state2.check_skipped("callback 2");

    // Resume first callback
    event1.try_trigger();
    state1.check_finished("callback 1");

    // Resume second callback - this should do nothing
    event2.try_trigger();
    state2.check_skipped("callback 2");
  }

} // namespace
