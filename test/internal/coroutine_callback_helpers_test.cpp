#include "mrs_lib/internal/coroutine_callback_helpers.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <stdexcept>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

#include "mrs_lib/coro/task.hpp"
#include "mrs_lib/utility/callback.hpp"

#include "mrs_lib_testing/ros_fixtures.hpp"


namespace
{

  using mrs_lib::coro_callback_tags::Reentrant;
  using mrs_lib::internal::NoCallbackGroupTag;

  mrs_lib::Task<> co_set_true(std::shared_ptr<bool> val)
  {
    if (val == nullptr)
    {
      throw std::logic_error("val must not be nullptr.");
    }
    *val = true;
    co_return;
  }

  std::shared_ptr<rclcpp::CallbackGroup> create_callback_group(rclcpp::CallbackGroupType type)
  {
    return std::make_shared<rclcpp::CallbackGroup>(type, rclcpp::contexts::get_global_default_context());
  }

  class IsCallbackGroupCoroCompatible : public mrs_lib_testing::RosInitAndShutdownFixture
  {
  };

  TEST_F(IsCallbackGroupCoroCompatible, Nullptr)
  {
    EXPECT_FALSE(mrs_lib::internal::is_callback_group_coro_compatible(nullptr));
  }

  TEST_F(IsCallbackGroupCoroCompatible, MutuallyExclusive)
  {
    auto callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    EXPECT_FALSE(mrs_lib::internal::is_callback_group_coro_compatible(callback_group));
  }

  TEST_F(IsCallbackGroupCoroCompatible, Reentrant)
  {
    auto callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    EXPECT_TRUE(mrs_lib::internal::is_callback_group_coro_compatible(callback_group));
  }

  class RequireCallbackGroupCoroCompatible : public mrs_lib_testing::RosInitAndShutdownFixture
  {
  };

  TEST_F(RequireCallbackGroupCoroCompatible, Nullptr)
  {
    EXPECT_THROW(mrs_lib::internal::require_callback_group_coro_compatible(nullptr), std::logic_error);
  }

  TEST_F(RequireCallbackGroupCoroCompatible, MutuallyExclusive)
  {
    auto callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    EXPECT_THROW(mrs_lib::internal::require_callback_group_coro_compatible(callback_group), std::logic_error);
  }

  TEST_F(RequireCallbackGroupCoroCompatible, Reentrant)
  {
    auto callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    EXPECT_NO_THROW(mrs_lib::internal::require_callback_group_coro_compatible(callback_group));
  }

  class GetDetachedCoroCallbackLauncher : public mrs_lib_testing::RosInitAndShutdownFixture
  {
  };

  TEST_F(GetDetachedCoroCallbackLauncher, NotBound)
  {
    auto finished = std::make_shared<bool>(false);
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_set_true);
    static_assert(std::same_as<mrs_lib::CoroCallback<void(std::shared_ptr<bool>)>, decltype(callback)>);
    auto callback_launcher = mrs_lib::internal::get_detached_coro_callback_launcher(callback, NoCallbackGroupTag{});
    static_assert(std::same_as<std::function<void(std::shared_ptr<bool>)>, decltype(callback_launcher)>);

    EXPECT_FALSE(*finished);
    callback_launcher(finished);
    EXPECT_TRUE(*finished);
  }

  TEST_F(GetDetachedCoroCallbackLauncher, Bound)
  {
    auto finished = std::make_shared<bool>(false);
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_set_true, finished);
    static_assert(std::same_as<mrs_lib::CoroCallback<void()>, decltype(callback)>);
    auto callback_launcher = mrs_lib::internal::get_detached_coro_callback_launcher(callback, NoCallbackGroupTag{});
    static_assert(std::same_as<std::function<void()>, decltype(callback_launcher)>);

    EXPECT_FALSE(*finished);
    callback_launcher();
    EXPECT_TRUE(*finished);
  }

  TEST_F(GetDetachedCoroCallbackLauncher, WorksWithReentrantGroup)
  {
    auto callback_group = std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::Reentrant, rclcpp::contexts::get_global_default_context());

    auto finished = std::make_shared<bool>(false);
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_set_true, finished);
    static_assert(std::same_as<mrs_lib::CoroCallback<void()>, decltype(callback)>);
    auto callback_launcher = mrs_lib::internal::get_detached_coro_callback_launcher(callback, callback_group);
    static_assert(std::same_as<std::function<void()>, decltype(callback_launcher)>);

    EXPECT_FALSE(*finished);
    callback_launcher();
    EXPECT_TRUE(*finished);
  }

  TEST_F(GetDetachedCoroCallbackLauncher, ThrowsOnMutuallyExclusiveCallbackGroup)
  {
    auto callback_group = std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::MutuallyExclusive, rclcpp::contexts::get_global_default_context());

    auto finished = std::make_shared<bool>(false);
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_set_true, finished);
    static_assert(std::same_as<mrs_lib::CoroCallback<void()>, decltype(callback)>);
    EXPECT_THROW(mrs_lib::internal::get_detached_coro_callback_launcher(callback, callback_group), std::logic_error);
  }

  TEST_F(GetDetachedCoroCallbackLauncher, ThrowsOnNullCallbackGroup)
  {
    auto finished = std::make_shared<bool>(false);
    auto callback = mrs_lib::CoroCallback(Reentrant{}, co_set_true, finished);
    static_assert(std::same_as<mrs_lib::CoroCallback<void()>, decltype(callback)>);
    EXPECT_THROW(mrs_lib::internal::get_detached_coro_callback_launcher(callback, nullptr), std::logic_error);
  }

} // namespace
