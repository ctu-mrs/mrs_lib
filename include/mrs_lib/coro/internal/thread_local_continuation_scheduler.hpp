#ifndef MRS_LIB_CORO_INTERNAL_THREAD_LOCAL_CONTINUATION_SCHEDULER_HPP_
#define MRS_LIB_CORO_INTERNAL_THREAD_LOCAL_CONTINUATION_SCHEDULER_HPP_


#include <coroutine>

namespace mrs_lib::internal
{

  // This is a workaround to GCC generated code overflowing stack when using symmetric transfer

  void resume_coroutine(std::coroutine_handle<> handle);
  void schedule_coroutine_continuation(std::coroutine_handle<> handle);

  struct MoveToThreadLocalContinuationScheduler
  {
    MoveToThreadLocalContinuationScheduler() = default;
    ~MoveToThreadLocalContinuationScheduler() = default;
    MoveToThreadLocalContinuationScheduler(const MoveToThreadLocalContinuationScheduler&) = delete;
    MoveToThreadLocalContinuationScheduler& operator=(const MoveToThreadLocalContinuationScheduler&) = delete;
    MoveToThreadLocalContinuationScheduler(MoveToThreadLocalContinuationScheduler&&) = delete;
    MoveToThreadLocalContinuationScheduler& operator=(MoveToThreadLocalContinuationScheduler&&) = delete;

    // Always suspend the task
    bool await_ready() noexcept
    {
      return false;
    }

    // Move the task to the thread local scheduler
    void await_suspend(std::coroutine_handle<> task_handle) noexcept
    {
      resume_coroutine(task_handle);
    }

    // The task is moved, return nothing
    void await_resume() noexcept
    {
    }
  };

} // namespace mrs_lib::internal


#endif // MRS_LIB_CORO_INTERNAL_THREAD_LOCAL_CONTINUATION_SCHEDULER_HPP_
