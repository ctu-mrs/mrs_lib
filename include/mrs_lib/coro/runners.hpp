#ifndef MRS_LIB_CORO_RUNNERS_HPP_
#define MRS_LIB_CORO_RUNNERS_HPP_

#include <concepts>
#include <coroutine>

#include <mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp>
#include <mrs_lib/coro/task.hpp>
#include <stop_token>
#include <utility>


namespace mrs_lib::coro
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
        /**
         * @brief Construct the promise type, storing the associated stop token.
         */
        template <typename... Args>
        promise_type(std::stop_token stop_token, Args&&...) : stop_token_(std::move(stop_token))
        {
        }

        AsyncRun get_return_object()
        {
          return {};
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

        std::stop_token stop_token_;
      };

    private:
      AsyncRun() = default;
    };

    template <>
    struct CancellableContinuationFor<AsyncRun::promise_type>
    {
      static CancellableContinuation release_continuation(std::coroutine_handle<AsyncRun::promise_type>)
      {
        return {};
      };

      static std::stop_token get_token(std::coroutine_handle<AsyncRun::promise_type> handle)
      {
        return handle.promise().stop_token_;
      }
    };

    /**
     * @brief Internal helper to safely start coroutine task.
     *
     * @param stop_token The stop token is used by the internal::AsyncRun promise object.
     * @param task Coroutine to start
     * @param args Additional arguments to the coroutine
     *
     * All arguments are taken by value to ensure that they are copied into
     * the coroutine frame, thus preventing dangling references similarly
     * to `std::thread`.
     *
     * @see start_task
     */
    template <typename F, typename... Args>
      requires std::invocable<std::decay_t<F>, std::decay_t<Args>...> && std::same_as<Task<void>, std::invoke_result_t<std::decay_t<F>, std::decay_t<Args>...>>
    internal::AsyncRun start_task_impl(std::stop_token, F task, Args... args)
    {
      // The `decay-copy` of the arguments ensures that they are copied into
      // the coroutine frame, preventing dangling references like `std::thread`.
      co_await std::invoke(std::move(task), std::move(args)...);
    }

    /**
     * @brief Start execution of a task from outside of a coroutine.
     *
     * @param task The coroutine task to start.
     * @param args Additional arguments to the task.
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
      internal::start_task_impl(std::stop_token(), std::forward<F>(task), std::forward<Args>(args)...);
    }

    /**
     * @brief Start execution of a task from outside of a coroutine.
     *
     * @param token Stop token that can be used to interrupt execution of the task.
     * @param task The coroutine task to start.
     * @param args Additional arguments to the task.
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
    void start_task(std::stop_token token, F&& task, Args&&... args)
    {
      internal::start_task_impl(token, std::forward<F>(task), std::forward<Args>(args)...);
    }

  } // namespace internal

} // namespace mrs_lib::coro

#endif // MRS_LIB_CORO_RUNNERS_HPP_
