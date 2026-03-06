#ifndef MRS_LIB_CORO_TASK_HPP_
#define MRS_LIB_CORO_TASK_HPP_

#include <cassert>
#include <concepts>
#include <coroutine>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include <mrs_lib/coro/internal/attributes.hpp>

// Note on ownership semantics:
// Since we want to support cancellation at any point in the coroutine stacks,
// each coroutine is owned by the object responsible for resuming it. If the
// coroutine is running, it is responsible for it's own lifetime - it has to
// either suspend and become continuation of some other task thus transferring
// ownership or destruct itself once completed.

namespace mrs_lib
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

      void set_continuation(OwningCoroutineHandle<> continuation);

    private:
      OwningCoroutineHandle<> continuation_{std::noop_coroutine()};
    };

    /**
     * @brief A variant-like class for storing the result of non-void task.
     */
    template <typename T>
    class ResultStorage
    {
    private:
      enum class State
      {
        empty,
        value,
        exception,
      };

    public:
      constexpr ResultStorage() noexcept : state_(State::empty)
      {
      }

      /**
       * @brief Store result of task.
       *
       * This can only ve called once and not if set_exception was called.
       */
      constexpr void set_value(T&& val) noexcept(std::is_nothrow_move_constructible_v<T>)
      {
        assert(state_ == State::empty);
        std::construct_at(&value_, std::move(val));
        state_ = State::value;
      }

      /**
       * @brief Store exception into the result.
       *
       * This can only ve called once and not if set_value was called.
       */
      void set_exception(std::exception_ptr eptr) noexcept
      {
        assert(state_ == State::empty);
        std::construct_at(&exception_, std::move(eptr));
        state_ = State::exception;
      }

      /**
       * @brief Get previously stored result or exception.
       *
       * If this result contains a value, it is returned. Otherwise, if there is
       * an exception stored, it is thrown.
       *
       * Either set_value or set_exception must be called before calling this.
       */
      constexpr T get_value() &&
      {
        if (state_ == State::exception)
        {
          std::rethrow_exception(exception_);
        }
        assert(state_ == State::value);
        return std::move(value_);
      }

      constexpr ~ResultStorage()
      {
        switch (state_)
        {
        case State::empty:
          break;
        case State::value:
          std::destroy_at(&value_);
          break;
        case State::exception:
          std::destroy_at(&exception_);
          break;
        }
      }

    private:
      union
      {
        T value_;
        std::exception_ptr exception_;
      };
      State state_;
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

      std::coroutine_handle<> await_suspend(std::coroutine_handle<> continuation)
      {
        task_handle_.promise().set_continuation(OwningCoroutineHandle<>(continuation));
        return task_handle_;
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
  class [[nodiscard("Task is lazy and does not run until `co_await`ed.")]] MRS_LIB_INTERNAL_CORO_RETURN_TYPE MRS_LIB_INTERNAL_CORO_LIFETIMEBOUND Task
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
    Task(internal::OwningCoroutineHandle<promise_type> coroutine) : coroutine_(std::move(coroutine))
    {
    }

    internal::OwningCoroutineHandle<promise_type> coroutine_;

    friend class internal::PromiseType<T>;
  };

} // namespace mrs_lib

#ifndef MRS_LIB_CORO_TASK_IMPL_HPP_
#include <mrs_lib/coro/task.impl.hpp> // IWYU pragma: export
#endif

#endif // MRS_LIB_CORO_TASK_HPP_
