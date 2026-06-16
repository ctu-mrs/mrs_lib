#ifndef MRS_LIB_CORO_TASK_HPP_
#define MRS_LIB_CORO_TASK_HPP_

#include <cassert>
#include <concepts>
#include <coroutine>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include "mrs_lib/coro/internal/attributes.hpp"
#include "mrs_lib/coro/internal/continuation.hpp"
#include "mrs_lib/coro/internal/result_storage.hpp"
#include "mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp"

// Note on ownership semantics:
// Since we want to support cancellation at any point in the coroutine stacks,
// each coroutine is owned by the object responsible for resuming it. If the
// coroutine is running, it is responsible for it's own lifetime - it has to
// either suspend and become continuation of some other task thus transferring
// ownership or destruct itself once completed.

namespace mrs_lib::coro
{

  template <typename T = void>
    requires(std::same_as<T, std::remove_cvref_t<T>>)
  class Task;

  namespace internal
  {

    /**
     * @brief Deleter for std::unique_ptr that stores a coroutine handle.
     */
    template <typename T>
    struct CoroutineDestroyer
    {
      void operator()(std::coroutine_handle<T> handle)
      {
        handle.destroy();
      }
      using pointer = std::coroutine_handle<T>;
    };

    template <typename T = void>
    using OwningCoroutineHandle = std::unique_ptr<std::coroutine_handle<T>, CoroutineDestroyer<T>>;

    /**
     * @brief RAII class to destroy a coroutine at the end of a scope.
     */
    template <typename T>
    class DeferredCoroutineDestroyer
    {
    public:
      DeferredCoroutineDestroyer(std::coroutine_handle<T> handle) : handle_(handle)
      {
      }
      ~DeferredCoroutineDestroyer()
      {
        std::invoke(CoroutineDestroyer<T>{}, handle_);
      }
      DeferredCoroutineDestroyer(const DeferredCoroutineDestroyer&) = delete;
      DeferredCoroutineDestroyer& operator=(const DeferredCoroutineDestroyer&) = delete;
      DeferredCoroutineDestroyer(DeferredCoroutineDestroyer&&) = delete;
      DeferredCoroutineDestroyer& operator=(DeferredCoroutineDestroyer&&) = delete;

    private:
      std::coroutine_handle<T> handle_;
    };

    /**
     * @brief Base class for the task's promise type.
     *
     * This implements the promise type interface that is common for both void
     * and non-void tasks.
     */
    template <typename Derived>
    class BasePromiseType
    {
      /**
       * @brief Awaitable used for final_suspend of mrs_lib::Task
       *
       * This class is responsible for resuming continuation of the completed task.
       */
      class FinalAwaitable
      {
      public:
        FinalAwaitable() = default;
        ~FinalAwaitable() = default;
        FinalAwaitable(const FinalAwaitable&) = delete;
        FinalAwaitable& operator=(const FinalAwaitable&) = delete;
        FinalAwaitable(FinalAwaitable&&) = delete;
        FinalAwaitable& operator=(FinalAwaitable&&) = delete;

        // Always suspend the ending task
        bool await_ready() noexcept;

        // SYMMETRIC TRANSFER IS BROKEN IN GCC and can result in stack
        // overflow when many tasks complete synchronously.
        // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=100897
        // Because of this problem, the `await_suspend` uses the void signature
        // and resumes the continuation on a thread-local scheduler as a workaround.
        void await_suspend(std::coroutine_handle<Derived> task_handle) noexcept;

        // This should be unreachable - ended task should not be resumed
        void await_resume() noexcept;
      };

    public:
      // The task is lazy and will only start when awaited
      std::suspend_always initial_suspend();
      // The coroutine will be suspended and the continuation will be resumed
      FinalAwaitable final_suspend() noexcept;

      void set_continuation(CancellableContinuation continuation);

      CancellableContinuation release_continuation()
      {
        return std::exchange(continuation_, {});
      }

      std::stop_token get_token() const
      {
        return continuation_.get_token();
      }

    private:
      CancellableContinuation continuation_;
    };

    /**
     * @brief Promise type for non-void task.
     *
     * This is responsible for returning value from completed task.
     */
    template <typename T>
    class PromiseType : public BasePromiseType<PromiseType<T>>
    {
    public:
      Task<T> get_return_object();

      void return_value(T&& ret_val);

      void unhandled_exception();

      T get_value()
      {
        return std::move(result_).get_value();
      }

    private:
      ResultStorage<T> result_;
    };

    /**
     * @brief Promise type for void task.
     *
     * This is responsible for returning void from completed task.
     */
    template <>
    class PromiseType<void> : public BasePromiseType<PromiseType<void>>
    {
    public:
      Task<void> get_return_object();

      void return_void();

      void unhandled_exception();

      void get_value()
      {
        if (exception_)
        {
          std::rethrow_exception(exception_);
        }
      }

    private:
      std::exception_ptr exception_;
    };

    template <typename T>
    struct CancellableContinuationFor<PromiseType<T>>
    {
      static CancellableContinuation release_continuation(std::coroutine_handle<PromiseType<T>> handle)
      {
        PromiseType<T>& promise = handle.promise();
        return promise.release_continuation();
      };

      static std::stop_token get_token(std::coroutine_handle<PromiseType<T>> handle)
      {
        return handle.promise().get_token();
      }
    };

    /**
     * @brief Awaitable used to co_await other tasks.
     *
     * This is responsible for suspending the caller and registering it as
     * a continuation of the callee.
     */
    template <typename T>
    class TaskAwaitable
    {
      using Promise = Task<T>::promise_type;

    public:
      bool await_ready()
      {
        return false;
      }

      // SYMMETRIC TRANSFER IS BROKEN IN GCC and can result in stack
      // overflow when many tasks complete synchronously.
      // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=100897
      // Because of this problem, the `await_suspend` uses the void signature
      // and resumes the continuation on a thread-local scheduler as a workaround.
      template <typename CallerPromise>
      void await_suspend(std::coroutine_handle<CallerPromise> continuation)
      {
        task_handle_.promise().set_continuation(CancellableContinuation(continuation));
        schedule_coroutine_continuation(task_handle_);
      }

      T await_resume()
      {
        DeferredCoroutineDestroyer destroyer{this->task_handle_};
        return this->task_handle_.promise().get_value();
      }

      ~TaskAwaitable() = default;
      TaskAwaitable(const TaskAwaitable&) = delete;
      TaskAwaitable& operator=(const TaskAwaitable&) = delete;
      TaskAwaitable(TaskAwaitable&&) = delete;
      TaskAwaitable& operator=(TaskAwaitable&&) = delete;

    private:
      TaskAwaitable(std::coroutine_handle<Promise> task_handle) : task_handle_(task_handle)
      {
      }

      std::coroutine_handle<Promise> task_handle_;

      friend class Task<T>;
    };

  } // namespace internal

  /**
   * @brief Task type for creating coroutines.
   *
   * @tparam T Return type of the coroutine (default is void)
   *
   * Task is lazy coroutine, which means it must be `co_awaited` to start
   * executing.
   */
  template <typename T>
    requires(std::same_as<T, std::remove_cvref_t<T>>)
  class [[nodiscard("Task is lazy and does not run until `co_await`ed.")]] Task
  {
  public:
    using promise_type = internal::PromiseType<T>;

    ~Task() = default;
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    Task(Task&&) = delete;
    Task& operator=(Task&&) = delete;

    friend internal::TaskAwaitable<T> operator co_await(Task task)
    {
      return internal::TaskAwaitable<T>(task.coroutine_.release());
    }

  private:
    explicit Task(internal::OwningCoroutineHandle<promise_type> coroutine) : coroutine_(std::move(coroutine))
    {
    }

    internal::OwningCoroutineHandle<promise_type> coroutine_;

    friend class internal::PromiseType<T>;
  };

} // namespace mrs_lib::coro

namespace mrs_lib
{
  // Export mrs_lib::coro::Task directly into mrs_lib namespace since it is
  // likely to be used often.
  using coro::Task;

} // namespace mrs_lib

#ifndef MRS_LIB_CORO_TASK_IMPL_HPP_
#include "mrs_lib/coro/task.impl.hpp" // IWYU pragma: export
#endif

#endif // MRS_LIB_CORO_TASK_HPP_
