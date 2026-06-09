#include "mrs_lib/coro/cancellation.hpp"

namespace mrs_lib::coro
{

  namespace internal
  {

    bool GetTaskStopTokenAwaiter::await_ready()
    {
      return false;
    }

    bool GetTaskStopTokenAwaiter::await_suspend(CancellableContinuation continuation)
    {
      token_ = continuation.get_token();
      continuation.release();
      return false;
    }

    std::stop_token GetTaskStopTokenAwaiter::await_resume()
    {
      return std::move(token_);
    }


  } // namespace internal

  internal::GetTaskStopTokenAwaiter get_task_stop_token()
  {
    return {};
  }

} // namespace mrs_lib::coro
