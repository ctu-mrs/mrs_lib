#ifndef MRS_LIB_CORO_RUNNERS_HPP_
#define MRS_LIB_CORO_RUNNERS_HPP_

#include <concepts>
#include <coroutine>

#include <mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp>
#include <mrs_lib/coro/task.hpp>


namespace mrs_lib
{

  namespace internal
  {

    /**
     * @brief Coroutine type used to start asynchronous computation.
     *
     * Calling a coroutine that returns this type runs until the first
     * suspension inside the body. After that, it is up to the awaitables to
     * resume or cancel the started coroutine.
     */
    class AsyncRun
    {
    public:
      struct promise_type
      {
        AsyncRun get_return_object()
        {
          return AsyncRun();
        }
        MoveToThreadLocalContinuationScheduler initial_suspend()
        {
          return {};
        }
        std::suspend_never final_suspend() noexcept
        {
          return {};
        }
        void return_void()
        {
        }
        void unhandled_exception()
        {
          throw;
        }
      };

    private:
      AsyncRun() = default;
    };

    template <class T>
      requires std::convertible_to<T, std::decay_t<T>>
    constexpr std::decay_t<T> decay_copy(T&& value) noexcept(std::is_nothrow_convertible_v<T, std::decay_t<T>>)
    {
      return std::forward<T>(value);
    }

    template <typename F, typename... Args>
      requires std::invocable<std::decay_t<F>, std::decay_t<Args>...> && std::same_as<Task<void>, std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>>
    internal::AsyncRun start_task_impl(F&& task, Args&&... args)
    {
      // The `decay-copy` of the arguments ensures that they are copied into
      // the coroutine frame, preventing dangling references like `std::thread`.
      co_await std::invoke(decay_copy(std::forward<F>(task)), decay_copy(std::forward<Args>(args))...);
    }

    /**
     * @brief Start execution of a task from outside of a coroutine.
     *
     * This function starts the execution of the coroutine using the provided
     * arguments. The passed arguments are decay-copied into the coroutine frame
     * (similarly to std::thread).
     *
     * Using this function from user code is not usually necessary. It can be
     * avoided by `co_await`ing the tasks and using callbacks that support passing
     * coroutine callbacks.
     */
    template <typename F, typename... Args>
      requires std::invocable<std::decay_t<F>, std::decay_t<Args>...> && std::same_as<Task<void>, std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>>
    void start_task(F&& task, Args&&... args)
    {
      internal::start_task_impl(std::forward<F>(task), std::forward<Args>(args)...);
    }

  } // namespace internal

} // namespace mrs_lib

#endif // MRS_LIB_CORO_RUNNERS_HPP_
