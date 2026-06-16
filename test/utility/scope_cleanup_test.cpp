#include "mrs_lib/utility/scope_cleanup.hpp"

#include <gtest/gtest.h>


namespace
{

  TEST(MrsLibUtilityScopeCleanup, RunsWhenDestroyed)
  {
    bool triggered = false;
    {
      mrs_lib::ScopeCleanup cleanup_set_triggered([&] { triggered = true; });

      EXPECT_FALSE(triggered);
    }

    EXPECT_TRUE(triggered);
  }

  TEST(MrsLibUtilityScopeCleanup, DoesNotRunWhenCanceled)
  {
    bool triggered = false;
    {
      mrs_lib::ScopeCleanup cleanup_set_triggered([&] { triggered = true; });

      EXPECT_FALSE(triggered);

      cleanup_set_triggered.cancel();
    }

    EXPECT_FALSE(triggered);
  }

} // namespace
