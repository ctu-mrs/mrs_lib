#include "mrs_lib/utility/pimpl.hpp"

#include <gtest/gtest.h>

#include <concepts>
#include <memory>
#include <utility>


namespace
{

  static_assert(!std::copy_constructible<mrs_lib::Pimpl<int, std::unique_ptr<int>>>);
  static_assert(std::move_constructible<mrs_lib::Pimpl<int, std::unique_ptr<int>>>);

  static_assert(!std::copy_constructible<mrs_lib::Pimpl<int, std::shared_ptr<int>>>);
  static_assert(std::move_constructible<mrs_lib::Pimpl<int, std::shared_ptr<int>>>);

  // DOCS: BEGIN EXAMPLE
  class MyClass
  {
    class Impl;

  public:
    MyClass() : impl_(std::in_place_type_t<Impl>{}, 21)
    {
    }

    int calculate();

  private:
    mrs_lib::Pimpl<Impl> impl_;
  };

  class MyClass::Impl
  {
  public:
    Impl(int val) : val_(val)
    {
    }

    int calculate()
    {
      return val_ * 2;
    }

  private:
    int val_;
  };

  int MyClass::calculate()
  {
    return impl_->calculate();
  }
  // DOCS: END EXAMPLE

  TEST(UtilityPimpl, Example)
  {
    MyClass obj = MyClass();
    EXPECT_EQ(obj.calculate(), 42);
  }

  TEST(UtilityPimpl, BasicUsageUniquePtr)
  {
    mrs_lib::Pimpl<int> pimpl(std::make_unique<int>(42));
    static_assert(std::same_as<mrs_lib::Pimpl<int, std::unique_ptr<int>>, decltype(pimpl)>);
    ASSERT_FALSE(pimpl.valueless_after_move());
    EXPECT_EQ(*pimpl, 42);

    auto pimpl2 = std::move(pimpl);
    EXPECT_TRUE(pimpl.valueless_after_move());
    ASSERT_FALSE(pimpl2.valueless_after_move());
    EXPECT_EQ(*pimpl2, 42);
  }

  TEST(UtilityPimpl, BasicUsageSharedPtr)
  {
    mrs_lib::Pimpl<int, std::shared_ptr<int>> pimpl(std::make_shared<int>(42));
    ASSERT_FALSE(pimpl.valueless_after_move());
    EXPECT_EQ(*pimpl, 42);

    auto pimpl2 = std::move(pimpl);
    EXPECT_TRUE(pimpl.valueless_after_move());
    ASSERT_FALSE(pimpl2.valueless_after_move());
    EXPECT_EQ(*pimpl2, 42);
  }

} // namespace
