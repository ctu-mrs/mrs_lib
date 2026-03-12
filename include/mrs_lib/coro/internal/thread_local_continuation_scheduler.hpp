#ifndef MRS_LIB_CORO_INTERNAL_THREAD_LOCAL_CONTINUATION_SCHEDULER_HPP_
#define MRS_LIB_CORO_INTERNAL_THREAD_LOCAL_CONTINUATION_SCHEDULER_HPP_


#include <coroutine>

#include <mrs_lib/internal/version_macros.hpp>

namespace mrs_lib::MRS_LIB_INTERNAL_INLINE_API_V2 v2::internal
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

} // namespace mrs_lib::inline v2::internal

namespace mrs_lib::MRS_LIB_INTERNAL_INLINE_API_V1 v1::internal
{

  // Backport resume_coroutine to v1 interfaces
  using ::mrs_lib::v2::internal::resume_coroutine;

} // namespace mrs_lib::inline v1::internal

#endif // MRS_LIB_CORO_INTERNAL_THREAD_LOCAL_CONTINUATION_SCHEDULER_HPP_
