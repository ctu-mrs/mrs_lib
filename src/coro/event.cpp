#include "mrs_lib/coro/event.hpp"

#include <memory>
#include <coroutine>
#include <mutex>
#include <cassert>
#include <optional>
#include <utility>

#include "mrs_lib/coro/internal/continuation.hpp"
#include "mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp"


namespace mrs_lib::coro
{

  std::pair<Event, EventAwaitable> make_event()
  {
    auto state = std::make_shared<internal::EventState>();
    return {
        Event(state),
        EventAwaitable(internal::LowLevelEventAwaitable(state)),
    };
  }

  EventError::EventError(Type type) : std::logic_error(get_msg(type)), type_(type)
  {
  }

  const char* EventError::get_msg(Type type)
  {
    switch (type)
    {
    case Type::empty_awaitable_state:
      return "EventError: This awaitable is empty";
    }
    // Not as `default:` to show switch warning if some case is missed.
    return "EventError: Unknown";
  }

  namespace internal
  {

    void EventState::StopCallbackCallable::operator()()
    {
      assert(event_state != nullptr);
      event_state->try_cancel();
    }

    bool EventState::try_trigger()
    {
      std::coroutine_handle<> handle = nullptr;

      {
        using Status = internal::EventState::Status;
        std::lock_guard lock(mutex_);
        if (status_ != Status::unset)
        {
          return false;
        }
        status_ = Status::set;
        handle = continuation_.release();
      }

      if (handle)
      {
        internal::resume_coroutine_soon(handle);
      }

      return true;
    }

    bool EventState::try_cancel()
    {
      internal::CancellableContinuation continuation{};

      {
        using Status = internal::EventState::Status;
        std::lock_guard lock(mutex_);
        if (status_ != Status::unset)
        {
          return false;
        }
        status_ = Status::cancelled;
        continuation = std::exchange(continuation_, {});
      }

      continuation.cancel_and_destroy();

      return true;
    }

    bool EventState::add_continuation(CancellableContinuation continuation, bool token_cancelable, std::function<void()> callback)
    {
      std::unique_lock lock(mutex_);

      switch (status_)
      {
      case Status::unset: {
        // Event not yet triggered. Store the continuation and suspend.
        assert(continuation_ == nullptr);
        continuation_ = std::move(continuation);
        std::stop_token token = continuation_.get_token();
        // The callbacks may immediately trigger so we need to unlock the lock before it is called.
        lock.unlock();
        // The callback may cause all current handles to the EventState to be destroyed
        // This can happen when the callback registers a waker that can be woken by a different thread.
        // The waker can resume/cancel the event and thus destroy both the Event and
        // EventAwaitable instances that point to this state.
        // This can in turn cause undefined behavior when registering the stop callback.
        // To prevent this, we create a temporary owner of the event state, that
        // will allow us to safely set the stop callback.
        auto self_owner = shared_from_this();
        callback();
        if (token_cancelable)
        {
          // Since no other place interacts with the stop callback, it is safe to set it even when not holding the lock.
          stop_callback_.emplace(std::move(token), StopCallbackCallable{.event_state = this});
        }
        return true;
      }
      case Status::set:
        // Do not suspend, the event was already triggered.
        continuation.release();
        return false;
      case Status::cancelled:
        // Event cancelled. Unlock the state before destroying the continuation.
        lock.unlock();
        continuation.cancel_and_destroy();
        return true;
      }

      assert(false);
    }

    bool EventState::is_ready()
    {
      std::lock_guard lock(mutex_);
      return status_ == Status::set;
    }


    LowLevelEventAwaitable::Awaiter::Awaiter(std::shared_ptr<internal::EventState> state, bool token_cancellable, std::function<void()> callback)
        : state_(std::move(state)), is_token_cancellable_(token_cancellable), callback_(std::move(callback))
    {
    }

    bool LowLevelEventAwaitable::Awaiter::await_ready()
    {
      assert(state_ != nullptr);
      return state_->is_ready();
    }

    bool LowLevelEventAwaitable::Awaiter::await_suspend(CancellableContinuation continuation)
    {
      assert(state_ != nullptr);
      return state_->add_continuation(std::move(continuation), is_token_cancellable_, std::exchange(callback_, {}));
    }

    void LowLevelEventAwaitable::Awaiter::await_resume()
    {
    }


    LowLevelEventAwaitable::LowLevelEventAwaitable(LowLevelEventAwaitable&& other) noexcept : state_(std::exchange(other.state_, nullptr))
    {
    }

    LowLevelEventAwaitable& LowLevelEventAwaitable::operator=(LowLevelEventAwaitable&& other) noexcept
    {
      state_ = std::exchange(other.state_, nullptr);
      return *this;
    }

    auto LowLevelEventAwaitable::get_awaitable() && -> ImmediateAwaitable<Awaiter>
    {
      return std::move(*this).get_awaitable(StopTokenBehavior::respect, [] {});
    }


    auto LowLevelEventAwaitable::get_awaitable(StopTokenBehavior stop_token_behavior, std::function<void()> callback) && -> ImmediateAwaitable<Awaiter>
    {
      if (state_ == nullptr)
      {
        throw EventError(EventError::Type::empty_awaitable_state);
      }

      bool token_cancellable = stop_token_behavior == StopTokenBehavior::respect;
      return Awaiter(std::exchange(state_, nullptr), token_cancellable, std::exchange(callback, {}));
    }

    LowLevelEventAwaitable::LowLevelEventAwaitable(std::shared_ptr<internal::EventState> state) : state_(std::move(state))
    {
      assert(state_ != nullptr);
    }

    LowLevelEventAwaitable get_low_level_event_awaitable(EventAwaitable event_awaitable)
    {
      return std::move(event_awaitable).internal_awaitable_;
    }

  } // namespace internal

  Event::~Event()
  {
    try_cancel();
  }

  Event::Event(Event&& other) noexcept : state_(std::exchange(other.state_, nullptr))
  {
  }

  Event& Event::operator=(Event&& other) noexcept
  {
    state_ = std::exchange(other.state_, nullptr);
    return *this;
  }

  bool Event::try_trigger()
  {
    if (state_ == nullptr)
    {
      return false;
    }

    return state_->try_trigger();
  }

  bool Event::try_cancel()
  {
    if (state_ == nullptr)
    {
      return false;
    }

    return state_->try_cancel();
  }

  Event::Event(std::shared_ptr<internal::EventState> state) : state_(std::move(state))
  {
    assert(state_ != nullptr);
  }


  EventAwaitable::EventAwaitable(internal::LowLevelEventAwaitable internal_awaitable) : internal_awaitable_(std::move(internal_awaitable))
  {
  }

  auto EventAwaitable::wait() && -> internal::ImmediateAwaitable<internal::LowLevelEventAwaitable::Awaiter>
  {
    return std::move(internal_awaitable_).get_awaitable();
  }

} // namespace mrs_lib::coro
