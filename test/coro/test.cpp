#include <format>
#include <memory>

#include <gtest/gtest.h>

#include <mrs_lib/coro/runners.hpp>
#include <mrs_lib/coro/task.hpp>

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//// Static assert tests                                                    ////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace
{
  struct PotentiallyThrowingMoveOnly
  {
    ~PotentiallyThrowingMoveOnly() = default;
    PotentiallyThrowingMoveOnly(const PotentiallyThrowingMoveOnly&) = delete;
    PotentiallyThrowingMoveOnly& operator=(const PotentiallyThrowingMoveOnly&) = delete;
    PotentiallyThrowingMoveOnly(PotentiallyThrowingMoveOnly&&) noexcept(false)
    {
      throw std::logic_error("");
    };
    PotentiallyThrowingMoveOnly& operator=(PotentiallyThrowingMoveOnly&&) noexcept(false)
    {
      throw std::logic_error("");
    };
  };

  static_assert(noexcept(std::declval<mrs_lib::internal::ResultStorage<int>>().set_value(10)),
                "ResultStorage::set_value should be noexcept when stored type is noexcept move constructible");
  static_assert(noexcept(std::declval<mrs_lib::internal::ResultStorage<std::string>>().set_value(std::declval<std::string>())),
                "ResultStorage::set_value should be noexcept when stored type is noexcept move constructible");
  static_assert(!noexcept(std::declval<mrs_lib::internal::ResultStorage<PotentiallyThrowingMoveOnly>>().set_value(std::declval<PotentiallyThrowingMoveOnly>())),
                "ResultStorage::set_value should NOT be noexcept when stored type is NOT noexcept move constructible");

  static_assert(noexcept(std::declval<mrs_lib::internal::ResultStorage<int>>().set_exception(std::declval<std::exception_ptr>())),
                "ResultStorage::set_exception should be noexcept");
  static_assert(noexcept(std::declval<mrs_lib::internal::ResultStorage<std::string>>().set_exception(std::declval<std::exception_ptr>())),
                "ResultStorage::set_exception should be noexcept");
  static_assert(noexcept(std::declval<mrs_lib::internal::ResultStorage<PotentiallyThrowingMoveOnly>>().set_exception(std::declval<std::exception_ptr>())),
                "ResultStorage::set_exception should be noexcept");

  consteval bool result_storage_test_default_ctor_dtor()
  {
    mrs_lib::internal::ResultStorage<int> storage;

    return true;
  }
  static_assert(result_storage_test_default_ctor_dtor());

  consteval bool result_storage_test_int()
  {
    mrs_lib::internal::ResultStorage<int> storage;
    storage.set_value(10);
    assert(std::move(storage).get_value() == 10);

    return true;
  }
  static_assert(result_storage_test_int());

  consteval bool result_storage_test_string()
  {
    std::string s = "asdf";
    mrs_lib::internal::ResultStorage<std::string> storage;
    storage.set_value(std::move(s));
    assert(std::move(storage).get_value() == "asdf");

    return true;
  }
  static_assert(result_storage_test_string());

} // namespace

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//// Runtime tests                                                          ////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace
{
  //////////////////////////////////////////////////////////////////////////////
  // MrsLibCoro                                                               //
  //////////////////////////////////////////////////////////////////////////////

  constexpr std::string_view test_exception_message = "Expected test exception";
  constexpr std::string_view test_not_thrown_exception_message = "This should not be thrown";

  [[noreturn]] void throw_expected_exception()
  {
    throw std::logic_error(std::string(test_exception_message));
  }

  mrs_lib::Task<> co_takes_val(int)
  {
    co_return;
  };

  mrs_lib::Task<> co_takes_lval_ref(int&)
  {
    co_return;
  };

  mrs_lib::Task<> co_takes_rval_ref(int&&)
  {
    co_return;
  };

  mrs_lib::Task<> co_takes_move_only(std::unique_ptr<int>)
  {
    co_return;
  };

  mrs_lib::Task<> co_takes_move_only_lval_ref(std::unique_ptr<int>&)
  {
    co_return;
  };

  mrs_lib::Task<> co_takes_move_only_rval_ref(std::unique_ptr<int>&&)
  {
    co_return;
  };

  mrs_lib::Task<> co_set_42(int& out)
  {
    out = 42;
    co_return;
  }

  mrs_lib::Task<int> co_42()
  {
    co_return 42;
  }

  mrs_lib::Task<std::unique_ptr<int>> co_uptr_42()
  {
    auto val = std::make_unique<int>(42);
    co_return val;
  }

  mrs_lib::Task<int> co_2_times_42()
  {
    auto a = co_await co_42();
    auto b = co_await co_uptr_42();
    co_return a + (*b);
  }

  mrs_lib::Task<> co_throws_logic_error()
  {
    throw_expected_exception();
    co_return;
  }

  mrs_lib::Task<> co_call_throwning(size_t levels)
  {
    if (levels == 0)
    {
      co_await co_throws_logic_error();
    } else
    {
      co_await co_call_throwning(levels - 1);
    }
  }

  mrs_lib::Task<int> co_throws_logic_error_non_void()
  {
    throw_expected_exception();
    co_return 42;
  }

  mrs_lib::Task<int> co_call_throwning_non_void(size_t levels)
  {
    if (levels == 0)
    {
      co_return co_await co_throws_logic_error_non_void();
    } else
    {
      co_return co_await co_call_throwning_non_void(levels - 1);
    }
  }

  template <typename R, typename... FuncArgs, typename... Args>
    requires std::invocable<mrs_lib::Task<R> (*)(FuncArgs...), Args...>
  mrs_lib::Task<bool> co_check_throws(mrs_lib::Task<R> (*coroutine)(FuncArgs...), Args&&... args)
  {
    bool expected_error = false;
    try
    {
      co_await coroutine(std::forward<Args>(args)...);
    }
    catch (std::logic_error& e)
    {
      expected_error = e.what() == test_exception_message;
      if (!expected_error)
      {
        std::cout << "Expected error msg: '" << test_exception_message << "'\n";
        std::cout << "Observed error msg: '" << e.what() << "'\n";
      }
    }
    co_return expected_error;
  }

  struct ErrorOnMove
  {
    ErrorOnMove() = default;
    ErrorOnMove(const ErrorOnMove&) = delete;
    ErrorOnMove& operator=(const ErrorOnMove&) = delete;
    ErrorOnMove(ErrorOnMove&&) noexcept(false)
    {
      throw_expected_exception();
    }
    ErrorOnMove& operator=(ErrorOnMove&&) noexcept(false)
    {
      throw_expected_exception();
    }
  };

  mrs_lib::Task<ErrorOnMove> co_get_error_on_move()
  {
    co_return ErrorOnMove{};
  }


  mrs_lib::Task<std::exception_ptr> co_get_exception_ptr()
  {
    co_return std::make_exception_ptr(std::logic_error(std::string(test_not_thrown_exception_message)));
  }

  mrs_lib::Task<bool> check_get_exception_ptr()
  {
    std::exception_ptr eptr = nullptr;
    try
    {
      eptr = co_await co_get_exception_ptr();
    }
    catch (std::exception& e)
    {
      std::cout << std::format("This should not throw. Caught exception: \n", e.what());
    }

    if (eptr == nullptr)
    {
      std::cout << "Error: exception_ptr is empty!\n";
      co_return false;
    }

    bool success = false;

    try
    {
      std::rethrow_exception(eptr);
    }
    catch (std::logic_error& e)
    {
      success = e.what() == test_not_thrown_exception_message;
    }

    co_return success;
  }

  TEST(MrsLibCoro, StartTaskArgsValueCategories)
  {
    // This test checks that start task compiles with all of these signatures.

    int lval_int = 42;
    std::unique_ptr<int> lval_uptr = std::make_unique<int>(42);
    // function reference
    mrs_lib::internal::start_task(co_takes_val, 42);
    mrs_lib::internal::start_task(co_takes_lval_ref, std::ref(lval_int));
    mrs_lib::internal::start_task(co_takes_rval_ref, 42);
    mrs_lib::internal::start_task(co_takes_move_only, std::make_unique<int>(42));
    mrs_lib::internal::start_task(co_takes_move_only_lval_ref, std::ref(lval_uptr));
    mrs_lib::internal::start_task(co_takes_move_only_rval_ref, std::make_unique<int>(42));
    // function pointer
    mrs_lib::internal::start_task(&co_takes_val, 42);
    mrs_lib::internal::start_task(&co_takes_lval_ref, std::ref(lval_int));
    mrs_lib::internal::start_task(&co_takes_rval_ref, 42);
    mrs_lib::internal::start_task(&co_takes_move_only, std::make_unique<int>(42));
    mrs_lib::internal::start_task(&co_takes_move_only_lval_ref, std::ref(lval_uptr));
    mrs_lib::internal::start_task(&co_takes_move_only_rval_ref, std::make_unique<int>(42));
  }

  TEST(MrsLibCoro, StartCopyDecays)
  {
    struct Func
    {
      mrs_lib::Task<> operator()(int&& arg)
      {
        arg = 24;
        co_return;
      }
      mrs_lib::Task<> operator()(int& arg)
      {
        arg = 42;
        co_return;
      }
    };

    int val = 0;

    // This call should copy the value and invoke the `int &&` overload
    mrs_lib::internal::start_task(Func{}, val);

    EXPECT_EQ(val, 0);

    // The reference wrapper should be able to keep the reference and call the `int&` overload
    mrs_lib::internal::start_task(Func{}, std::ref(val));

    EXPECT_EQ(val, 42);
  }

  TEST(MrsLibCoro, StartTaskLambda)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }

  TEST(MrsLibCoro, StartTaskFuncRef)
  {
    int result = 0;

    // No ampersand in front of function name - passing function reference
    mrs_lib::internal::start_task(co_set_42, std::ref(result));

    EXPECT_EQ(result, 42);
  }

  TEST(MrsLibCoro, StartTaskFuncPtr)
  {
    int result = 0;

    // Ampersand in front of function name - passing function pointer
    mrs_lib::internal::start_task(&co_set_42, std::ref(result));

    EXPECT_EQ(result, 42);
  }

  TEST(MrsLibCoro, CoroIsLazy)
  {
    int result = 0;

    // The function is called but not co-awaited. Since the task is lazy, the
    // value in result should not be changed by this.
    [[maybe_unused]] auto task = co_set_42(result);

    EXPECT_EQ(result, 0);
  }


  TEST(MrsLibCoro, ReturnInt)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          int val = co_await co_42();
          EXPECT_EQ(val, 42);
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }

  TEST(MrsLibCoro, ReturnUniquePtr)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          std::unique_ptr<int> val = co_await co_uptr_42();
          EXPECT_EQ(*val, 42);
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }

  TEST(MrsLibCoro, ChainedCoroutines)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          int val = co_await co_2_times_42();
          EXPECT_EQ(val, 84);
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }

  TEST(MrsLibCoro, ExceptionPropagationVoid)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          bool val = co_await co_check_throws(co_throws_logic_error);
          EXPECT_TRUE(val);
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }

  TEST(MrsLibCoro, MultiLevelExceptionPropagationVoid)
  {
    for (size_t level = 1; level < 10; level++)
    {
      bool finished = false;

      mrs_lib::internal::start_task(
          [](bool& finished, size_t level) -> mrs_lib::Task<> {
            bool val = co_await co_check_throws(co_call_throwning, level);
            EXPECT_TRUE(val);
            finished = true;
            co_return;
          },
          std::ref(finished), level);

      EXPECT_TRUE(finished);
    }
  }

  TEST(MrsLibCoro, ExceptionPropagationNonVoid)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          bool val = co_await co_check_throws(co_throws_logic_error_non_void);
          EXPECT_TRUE(val);
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }

  TEST(MrsLibCoro, MultiLevelExceptionPropagationNonVoid)
  {
    for (size_t level = 1; level < 10; level++)
    {
      bool finished = false;

      mrs_lib::internal::start_task(
          [](bool& finished, size_t level) -> mrs_lib::Task<> {
            bool val = co_await co_check_throws(co_call_throwning_non_void, level);
            EXPECT_TRUE(val);
            finished = true;
            co_return;
          },
          std::ref(finished), level);

      EXPECT_TRUE(finished);
    }
  }

  TEST(MrsLibCoro, ErrorOnReturn)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          bool val = co_await co_check_throws(co_get_error_on_move);
          EXPECT_TRUE(val);
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }

  TEST(MrsLibCoro, ReturnExceptionPtr)
  {
    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished) -> mrs_lib::Task<> {
          bool val = co_await check_get_exception_ptr();
          EXPECT_TRUE(val);
          finished = true;
          co_return;
        },
        std::ref(finished));

    EXPECT_TRUE(finished);
  }


  //////////////////////////////////////////////////////////////////////////////
  // MrsLibCoroLoops                                                          //
  //////////////////////////////////////////////////////////////////////////////

  class MrsLibCoroLoops : public ::testing::TestWithParam<size_t>
  {
  };

  mrs_lib::Task<size_t> co_get_1()
  {
    co_return 1;
  }

  const volatile auto get_one_coro_ptr = &co_get_1;

  mrs_lib::Task<size_t> co_test_long_loop(size_t iterations)
  {
    size_t sum = 0;
    for (size_t i = 0; i < iterations; ++i)
    {
      // Using volatile function pointer to remove some optimizations
      sum += co_await (*get_one_coro_ptr)();
    }
    co_return sum;
  }

  // This test checks that long chains of synchronous completions work correctly.
  // On incorrect implementations, long chains of synchronous completions may
  // cause stack overflow. Thus, this test runs long loops of synchronously
  // completing tasks to check for this stack overflow.
  TEST_P(MrsLibCoroLoops, LongLoop)
  {
    const size_t iterations = GetParam();

    bool finished = false;

    mrs_lib::internal::start_task(
        [](bool& finished, size_t iterations) -> mrs_lib::Task<> {
          size_t val = co_await co_test_long_loop(iterations);
          EXPECT_EQ(val, iterations);
          finished = true;
          co_return;
        },
        std::ref(finished), iterations);

    EXPECT_TRUE(finished);
  }

  INSTANTIATE_TEST_SUITE_P(I, MrsLibCoroLoops, ::testing::Values(1'000, 10'000, 100'000, 1'000'000, 10'000'000, 100'000'000),
                           [](const testing::TestParamInfo<MrsLibCoroLoops::ParamType>& info) { return std::format("{}_iters", info.param); });

} // namespace
