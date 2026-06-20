// RUNNER HEADER START
// {
//   "should_succeed": false,
//   "expected_diagnostics": [
//     "delete",
//     "// This line should cause error."
//   ]
// }
// RUNNER HEADER END


#include "mrs_lib/coro/task.hpp"

mrs_lib::Task<int> co_get_int(const int& val)
{
  co_return val * 2;
}

mrs_lib::Task<void> test()
{
  // This code should not compile as it leads to dangling reference.
  auto task = co_get_int(21);
  co_await std::move(task); // This line should cause error.
}
