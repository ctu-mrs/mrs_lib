#include "mrs_lib/utility/owning_mutex.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace
{

  // DOCS: BEGIN EXAMPLE
  struct PrinterData
  {
    std::size_t count = 1;
    std::string text = "hi";
  };

  class MyRepeatedPrinter
  {
  public:
    void set_count(std::size_t count)
    {
      // non const `data_` -> mutable guard
      auto guard = data_.acquire();
      guard->count = count;
    }

    void set_text(std::string text)
    {
      // non const `data_` -> mutable guard
      auto guard = data_.acquire();
      guard->text = std::move(text);
    }

    void print(std::ostream& os) const
    {
      // const `data_` -> const guard
      auto guard = data_.acquire();

      // guard->count += 1; // This would cause compilation error.
      //                    // We only get const access.

      for (std::size_t i = 0; i < guard->count; ++i)
      {
        os << guard->text << "\n";
      }
    }

  private:
    mrs_lib::OwningMutex<PrinterData> data_{};
  };
  // DOCS: END EXAMPLE

  TEST(UtilityOwningMutex, Example)
  {
    MyRepeatedPrinter printer;

    {
      std::ostringstream ss;
      printer.print(ss);
      EXPECT_EQ(ss.view(), "hi\n");
    }

    printer.set_count(3);

    {
      std::ostringstream ss;
      printer.print(ss);
      EXPECT_EQ(ss.view(), "hi\nhi\nhi\n");
    }

    printer.set_text("bye");

    {
      std::ostringstream ss;
      printer.print(ss);
      EXPECT_EQ(ss.view(), "bye\nbye\nbye\n");
    }
  }

  struct Data
  {
    bool bool_val;
    int int_val;
    float float_val;
    std::string string_val;

    friend bool operator==(const Data&, const Data&) = default;
  };

  Data get_data_1()
  {
    return {
        .bool_val = true,
        .int_val = 15,
        .float_val = 666.0f,
        .string_val = "initial_value",
    };
  }

  Data get_data_2()
  {
    return {
        .bool_val = false,
        .int_val = -15,
        .float_val = 333.0f,
        .string_val = "mutexed_value",
    };
  }

  class MockMutexAlreadyLockedError : std::logic_error
  {
  public:
    MockMutexAlreadyLockedError() : std::logic_error("Mock mutex already locked - Deadlock")
    {
    }
  };

  class MockMutex
  {
  public:
    MockMutex()
    {
      bool was_used = std::exchange(in_use_, true);
      if (was_used)
      {
        throw std::logic_error("This Mock mutex is already in use on this thread.");
      }
      locked_ = false;
      locked_count_ = 0;
    };
    ~MockMutex()
    {
      in_use_ = false;
    };

    MockMutex(const MockMutex&) = delete;
    MockMutex& operator=(const MockMutex&) = delete;
    MockMutex(MockMutex&&) = delete;
    MockMutex& operator=(MockMutex&&) = delete;

    static bool is_in_use()
    {
      return in_use_;
    }

    static bool is_locked()
    {
      return locked_;
    }

    static size_t get_locked_count()
    {
      return locked_count_;
    }

    void lock()
    {
      if (locked_)
      {
        throw MockMutexAlreadyLockedError();
      }
      locked_count_ += 1;
      locked_ = true;
    }

    void unlock()
    {
      if (!locked_)
      {
        throw std::logic_error("Unlocking unlocked mutex.");
      }
      locked_ = false;
    }

  private:
    inline static thread_local bool in_use_ = false;
    inline static thread_local bool locked_ = false;
    inline static thread_local size_t locked_count_ = 0;
  };

  TEST(UtilityOwningMutex, BasicUsageMockMutex)
  {
    ASSERT_FALSE(MockMutex::is_in_use());
    size_t expected_lock_count = 0;

    mrs_lib::OwningMutex<Data, MockMutex> data(get_data_1());

    EXPECT_TRUE(MockMutex::is_in_use());
    EXPECT_EQ(MockMutex::is_locked(), false);
    EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);

    auto copy_1 = data.load();
    expected_lock_count++;

    EXPECT_EQ(MockMutex::is_locked(), false);
    EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);
    EXPECT_EQ(copy_1, get_data_1());

    data.store(get_data_2());
    expected_lock_count++;

    EXPECT_EQ(MockMutex::is_locked(), false);
    EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);

    auto copy_2 = data.load();
    expected_lock_count++;

    EXPECT_EQ(MockMutex::is_locked(), false);
    EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);
    EXPECT_EQ(copy_2, get_data_2());

    {
      auto guard = data.acquire();
      expected_lock_count++;

      EXPECT_EQ(MockMutex::is_locked(), true);
      EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);

      EXPECT_EQ(guard->bool_val, get_data_2().bool_val);
      EXPECT_EQ(guard->int_val, get_data_2().int_val);
      EXPECT_EQ(guard->float_val, get_data_2().float_val);
      EXPECT_EQ(guard->string_val, get_data_2().string_val);

      guard->string_val = "";
    }

    EXPECT_EQ(MockMutex::is_locked(), false);
    EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);

    auto copy_3 = data.load();
    expected_lock_count++;

    EXPECT_EQ(MockMutex::is_locked(), false);
    EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);
    EXPECT_EQ(copy_3.string_val, "");

    {
      auto guard = data.acquire();
      expected_lock_count++;

      EXPECT_EQ(MockMutex::is_locked(), true);
      EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);

      EXPECT_EQ(*guard, copy_3);
      *guard = get_data_1();
    }

    auto copy_4 = data.load();
    expected_lock_count++;

    EXPECT_EQ(MockMutex::is_locked(), false);
    EXPECT_EQ(MockMutex::get_locked_count(), expected_lock_count);
    EXPECT_EQ(copy_4, get_data_1());
  }

  TEST(UtilityOwningMutex, BasicUsageDefaultMutex)
  {
    mrs_lib::OwningMutex<Data> data(get_data_1());

    auto copy_1 = data.load();

    EXPECT_EQ(copy_1, get_data_1());

    data.store(get_data_2());

    auto copy_2 = data.load();

    EXPECT_EQ(copy_2, get_data_2());

    {
      auto guard = data.acquire();

      EXPECT_EQ(guard->bool_val, get_data_2().bool_val);
      EXPECT_EQ(guard->int_val, get_data_2().int_val);
      EXPECT_EQ(guard->float_val, get_data_2().float_val);
      EXPECT_EQ(guard->string_val, get_data_2().string_val);

      guard->string_val = "";
    }

    auto copy_3 = data.load();

    EXPECT_EQ(copy_3.string_val, "");

    {
      auto guard = data.acquire();

      EXPECT_EQ(*guard, copy_3);
      *guard = get_data_1();
    }

    auto copy_4 = data.load();

    EXPECT_EQ(copy_4, get_data_1());
  }

  TEST(UtilityOwningMutex, RecursiveWithMockMutex)
  {
    ASSERT_FALSE(MockMutex::is_in_use());

    mrs_lib::OwningMutex<Data, MockMutex> data(get_data_1());

    auto guard_1 = data.acquire();

    EXPECT_THROW(auto guard_2 = data.acquire(), MockMutexAlreadyLockedError);
  }

  TEST(UtilityOwningMutex, RecursiveWithStdRecursiveMutex)
  {
    mrs_lib::OwningMutex<Data, std::recursive_mutex> data(get_data_1());

    auto guard_1 = data.acquire();
    auto guard_2 = data.acquire();
    *guard_2 = get_data_2();
    EXPECT_EQ(*guard_1, get_data_2());
  }

} // namespace
