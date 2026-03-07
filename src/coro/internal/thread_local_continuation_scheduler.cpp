#include <mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp>

#include <cassert>
#include <coroutine>
#include <cstddef>

namespace mrs_lib::internal
{

  namespace
  {

    // This is a workaround to GCC generated code overflowing stack when using symmetric transfer
    class ThreadLocalContinuationScheduler
    {
    public:
      static void resume_coroutine(std::coroutine_handle<> handle)
      {
        auto&& scheduler = get_thread_local_scheduler_();
        scheduler.run_until_suspend_(handle);
      }

      static void schedule_coroutine_continuation(std::coroutine_handle<> handle)
      {
        auto&& scheduler = get_thread_local_scheduler_();
        scheduler.set_continuation_(handle);
      }

      ~ThreadLocalContinuationScheduler() = default;
      ThreadLocalContinuationScheduler(const ThreadLocalContinuationScheduler&) = delete;
      ThreadLocalContinuationScheduler& operator=(const ThreadLocalContinuationScheduler&) = delete;
      ThreadLocalContinuationScheduler(ThreadLocalContinuationScheduler&&) = delete;
      ThreadLocalContinuationScheduler& operator=(ThreadLocalContinuationScheduler&&) = delete;

    private:
      static ThreadLocalContinuationScheduler& get_thread_local_scheduler_()
      {
        thread_local static ThreadLocalContinuationScheduler scheduler;
        return scheduler;
      }

      ThreadLocalContinuationScheduler() = default;

      void set_continuation_(std::coroutine_handle<> continuation)
      {
        assert(running);
        assert(released_id_ == stored_id_);
        continuation_ = continuation;
        stored_id_++;
      }

      void run_until_suspend_(std::coroutine_handle<> handle)
      {
        assert(!running);
        running = true;
        set_continuation_(handle);
        assert(released_id_ + 1 == stored_id_);
        while (released_id_ != stored_id_)
        {
          assert(released_id_ + 1 == stored_id_);
          released_id_++;
          continuation_.resume();
        }
        assert(running);
        running = false;
      }

      // Using unsigned ids that have defined overflow, removing the need to manually handle.
      size_t stored_id_ = 0;
      size_t released_id_ = 0;
      bool running = false;
      std::coroutine_handle<> continuation_;
    };

  } // namespace

  void resume_coroutine(std::coroutine_handle<> handle)
  {
    ThreadLocalContinuationScheduler::resume_coroutine(handle);
  }

  void schedule_coroutine_continuation(std::coroutine_handle<> handle)
  {
    ThreadLocalContinuationScheduler::schedule_coroutine_continuation(handle);
  }


} // namespace mrs_lib::internal
