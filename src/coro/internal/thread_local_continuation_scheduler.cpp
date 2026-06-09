#include "mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp"

#include <cassert>
#include <coroutine>
#include <cstddef>
#include <deque>

namespace mrs_lib::coro::internal
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

      static void resume_coroutine_soon(std::coroutine_handle<> handle)
      {
        auto&& scheduler = get_thread_local_scheduler_();
        scheduler.run_soon_until_suspend_(handle);
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

      void run_queue_()
      {
        assert(!running);
        running = true;
        while (coroutine_queue_.size() > 0)
        {
          std::coroutine_handle<> handle = coroutine_queue_.front();
          coroutine_queue_.pop_front();

          set_continuation_(handle);

          assert(released_id_ + 1 == stored_id_);
          while (released_id_ != stored_id_)
          {
            assert(released_id_ + 1 == stored_id_);
            released_id_++;
            continuation_.resume();
          }
        }
        assert(running);
        running = false;
      }

      void run_until_suspend_(std::coroutine_handle<> handle)
      {
        assert(!running);
        assert(coroutine_queue_.size() == 0);
        coroutine_queue_.push_back(handle);
        run_queue_();
      }

      void run_soon_until_suspend_(std::coroutine_handle<> handle)
      {
        if (!running)
        {
          run_until_suspend_(handle);
        } else
        {
          coroutine_queue_.push_back(handle);
        }
      }

      // Using unsigned ids that have defined overflow, removing the need to manually handle.
      size_t stored_id_ = 0;
      size_t released_id_ = 0;
      bool running = false;
      std::coroutine_handle<> continuation_;

      std::deque<std::coroutine_handle<>> coroutine_queue_;
    };

  } // namespace

  void resume_coroutine_soon(std::coroutine_handle<> handle)
  {
    ThreadLocalContinuationScheduler::resume_coroutine_soon(handle);
  }

  void schedule_coroutine_continuation(std::coroutine_handle<> handle)
  {
    ThreadLocalContinuationScheduler::schedule_coroutine_continuation(handle);
  }


} // namespace mrs_lib::coro::internal
