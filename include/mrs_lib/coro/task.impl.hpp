#ifndef MRS_LIB_CORO_TASK_IMPL_HPP_
#define MRS_LIB_CORO_TASK_IMPL_HPP_

#include <mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp>
#include <mrs_lib/coro/task.hpp>
#include <mrs_lib/internal/version_macros.hpp>

namespace mrs_lib::MRS_LIB_INTERNAL_INLINE_API_V2 v2
{
  namespace internal
  {

    template <typename Derived>
    inline bool BasePromiseType<Derived>::FinalAwaitable::await_ready() noexcept
    {
      return false;
    }

    template <typename Derived>
    inline void BasePromiseType<Derived>::FinalAwaitable::await_suspend(std::coroutine_handle<Derived> task_handle) noexcept
    {
      // Destructor of task would destroy the continuation so we need to release the continuation
      schedule_coroutine_continuation(task_handle.promise().continuation_.release());
    }

    template <typename Derived>
    inline void BasePromiseType<Derived>::FinalAwaitable::await_resume() noexcept
    {
      assert(false);
    }

    template <typename Derived>
    inline std::suspend_always BasePromiseType<Derived>::initial_suspend()
    {
      return {};
    }

    template <typename Derived>
    inline auto BasePromiseType<Derived>::final_suspend() noexcept -> FinalAwaitable
    {
      return {};
    }

    template <typename Derived>
    inline void BasePromiseType<Derived>::set_continuation(OwningCoroutineHandle<> continuation)
    {
      continuation_ = std::move(continuation);
    }

    template <typename T>
    Task<T> PromiseType<T>::get_return_object()
    {
      return Task<T>(OwningCoroutineHandle<PromiseType>(std::coroutine_handle<PromiseType>::from_promise(*this)));
    }

    template <typename T>
    void PromiseType<T>::return_value(T&& ret_val)
    {
      result_.set_value(std::move(ret_val));
    }

    template <typename T>
    void PromiseType<T>::unhandled_exception()
    {
      result_.set_exception(std::current_exception());
    }

    inline Task<void> PromiseType<void>::get_return_object()
    {
      return Task<void>(OwningCoroutineHandle<PromiseType>(std::coroutine_handle<PromiseType>::from_promise(*this)));
    }

    inline void PromiseType<void>::return_void()
    {
    }

    inline void PromiseType<void>::unhandled_exception()
    {
      exception_ = std::current_exception();
    }

  } // namespace internal
} // namespace mrs_lib::inline v2


#endif // MRS_LIB_CORO_TASK_IMPL_HPP_
