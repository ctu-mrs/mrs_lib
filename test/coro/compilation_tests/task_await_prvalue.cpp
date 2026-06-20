// RUNNER HEADER START
// {
//   "should_succeed": true,
//   "expected_diagnostics": [
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
  co_await co_get_int(21); // This line should not cause any errors.
}
