#ifndef MRS_LIB_CORO_CANCELLATION_HPP_
#define MRS_LIB_CORO_CANCELLATION_HPP_


#include <cassert>
#include <coroutine>
#include <stop_token>

#include "mrs_lib/coro/internal/continuation.hpp"


namespace mrs_lib::coro
{

  namespace internal
  {

    class [[nodiscard]] GetTaskStopTokenAwaiter
    {
    public:
      bool await_ready();

      template <typename T>
      bool await_suspend(std::coroutine_handle<T> handle)
      {
        return await_suspend(CancellableContinuation(handle));
      }
      bool await_suspend(CancellableContinuation continuation);

      std::stop_token await_resume();

    private:
      std::stop_token token_;
    };

  } // namespace internal

  /**
   * @brief Get awaitable that will return stop token associated with the current coroutine.
   *
   * @note Return value must be awaited.
   */
  internal::GetTaskStopTokenAwaiter get_task_stop_token();

} // namespace mrs_lib::coro

#endif // MRS_LIB_CORO_CANCELLATION_HPP_
