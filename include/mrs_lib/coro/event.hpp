#ifndef MRS_LIB_CORO_EVENT_HPP_
#define MRS_LIB_CORO_EVENT_HPP_


#include <memory>
#include <coroutine>
#include <mutex>
#include <cassert>
#include <optional>
#include <utility>

#include "mrs_lib/coro/internal/continuation.hpp"
#include "mrs_lib/coro/internal/immediate_awaitable.hpp"


namespace mrs_lib::coro
{

  class Event;
  class EventAwaitable;

  /**
   * @brief Create Event and its bound awaitable.
   *
   * Together, the Event and EventAwaitable form a onedirectional oneshot
   * communication channel.
   *
   * @see mrs_lib::coro::Event
   * @see mrs_lib::coro::EventAwaitable
   */
  std::pair<Event, EventAwaitable> make_event();

  /**
   * @brief Exception thrown on incorrect use of Event/EventAwaitable.
   */
  class EventError : std::logic_error
  {
  public:
    enum class Type
    {
      empty_awaitable_state,
    };

    EventError(Type type);

  private:
    static const char* get_msg(Type type);

    Type type_;
  };

  namespace internal
  {

    /**
     * @brief Internal class for handling events.
     */
    class EventState
    {
    private:
      enum class Status
      {
        unset,
        set,
        cancelled,
      };

      /**
       * @brief Callable used in stop_callback to cancel the wait.
       */
      struct StopCallbackCallable
      {
        EventState* event_state;

        void operator()();
      };

    public:
      /**
       * @brief Try triggering the event state.
       * @return true if succeeded, false otherwise
       *
       * The call will fail (return false) if the event was already triggered or cancelled.
       */
      bool try_trigger();

      /**
       * @brief Try cancelling the event state.
       * @return true if succeeded, false otherwise
       *
       * The call will fail (return false) if the event was already triggered or cancelled.
       */
      bool try_cancel();

      /**
       * @brief Store or resume the continuation as target for this event.
       *
       * @param continuation Continuation to use for the event.
       * @param token_cancelable Whether to respect the stop token of the continuation.
       * @param callback Callback to run when the coroutine suspends.
       * @return value to return from await_suspend
       *
       * If the state was not yet triggered nor cancelled, the continuation is
       * stored and resumed/canceled when the respective event comes.
       * The @p callback is called only in this case.
       *
       * If the event was already triggered, it will return to not suspend.
       *
       * If the event was already cancelled, the continuation will be cancelled immediately.
       */
      bool add_continuation(CancellableContinuation continuation, bool token_cancelable, std::function<void()> callback);

      /**
       * @brief Check if the event was already triggered.
       *
       * @return true if event was already triggered, false otherwise.
       *
       * @note If the event was cancelled, this will return false.
       */
      bool is_ready();

    private:
      std::mutex mutex_{};
      Status status_ = Status::unset;
      internal::CancellableContinuation continuation_{};

      std::optional<std::stop_callback<StopCallbackCallable>> stop_callback_{};
    };

    /**
     * @brief Internal awaitable used to wait for events.
     *
     * This should likely not be used outside of coroutine library functions.
     */
    class LowLevelEventAwaitable
    {
    public:
      class [[nodiscard]] Awaiter
      {
      public:
        explicit Awaiter(std::shared_ptr<internal::EventState> state, bool token_cancellable, std::function<void()> callback);

        bool await_ready();
        template <typename T>
        bool await_suspend(std::coroutine_handle<T> handle)
        {
          return await_suspend(CancellableContinuation(handle));
        }
        bool await_suspend(CancellableContinuation continuation);
        void await_resume();

      private:
        std::shared_ptr<internal::EventState> state_;
        bool is_token_cancellable_ = true;
        std::function<void()> callback_;
      };

      enum class StopTokenBehavior
      {
        respect,
        ignore,
      };


      ~LowLevelEventAwaitable() = default;

      LowLevelEventAwaitable(const LowLevelEventAwaitable&) = delete;
      LowLevelEventAwaitable& operator=(const LowLevelEventAwaitable&) = delete;
      LowLevelEventAwaitable(LowLevelEventAwaitable&& other) noexcept;
      LowLevelEventAwaitable& operator=(LowLevelEventAwaitable&& other) noexcept;

      /**
       * @brief Create awaitable for this event with default configuration.
       */
      ImmediateAwaitable<Awaiter> get_awaitable() &&;

      /**
       * @brief Create awaitable for this event.
       *
       * @param stop_token_behavior Whether to respect stop token request to cancel the await.
       * @param callback Callback to run during suspend.
       *
       * The callback is only run if the coroutine is suspended.
       *
       * The callback can be used to register a waker that will start the coroutine once ready.
       *
       * @warning The passed function should not reference local state of the
       * coroutine as it may be destroyed during the callback execution.
       */
      ImmediateAwaitable<Awaiter> get_awaitable(StopTokenBehavior stop_token_behavior, std::function<void()> callback) &&;

    private:
      explicit LowLevelEventAwaitable(std::shared_ptr<internal::EventState> state);

      std::shared_ptr<internal::EventState> state_;

      friend std::pair<Event, EventAwaitable> mrs_lib::coro::make_event();
    };

    /**
     * @brief Get LowLevelEventAwaitable from EventAwaitable.
     */
    LowLevelEventAwaitable get_low_level_event_awaitable(EventAwaitable event_awaitable);

  } // namespace internal

  /**
   * @brief Event that can be used to trigger coroutines.
   *
   * This is the write part of a onedirectional oneshot communication channel.
   * To create the pair, use mrs_lib::coro::make_event.
   *
   * @see mrs_lib::coro::make_event
   * @see mrs_lib::coro::EventAwaitable
   */
  class Event
  {
  public:
    ~Event();

    Event(const Event&) = delete;
    Event& operator=(const Event&) = delete;

    Event(Event&& other) noexcept;
    Event& operator=(Event&& other) noexcept;

    /**
     * @brief Try triggering the event.
     * @return true if succeeded, false otherwise
     *
     * The call will fail (return false) if the event was already triggered or cancelled.
     */
    bool try_trigger();

    /**
     * @brief Try cancelling the event.
     * @return true if succeeded, false otherwise
     *
     * The call will fail (return false) if the event was already triggered or cancelled.
     */
    bool try_cancel();

  private:
    explicit Event(std::shared_ptr<internal::EventState> state);

    std::shared_ptr<internal::EventState> state_;

    friend std::pair<Event, EventAwaitable> make_event();
  };

  /**
   * @brief Event awaitable that can be awaited by coroutines, resuming them once triggered.
   *
   * This is the read part of a onedirectional oneshot communication channel.
   * To create the pair, use mrs_lib::coro::make_event.
   *
   * @see mrs_lib::coro::make_event
   * @see mrs_lib::coro::Event
   */
  class EventAwaitable
  {
  public:
    /**
     * @brief Create EventAwaitable from a low level awaitable.
     *
     * This function should not be used in user code.
     * Instead, use mrs_lib::coro::make_event.
     */
    explicit EventAwaitable(internal::LowLevelEventAwaitable internal_awaitable);

    /**
     * @brief Wait for the event to trigger / cancel.
     *
     * Result of this function must be awaited.
     *
     * @note Every event can only be awaited once.
     * To help enforce that behavior, this method is rvalue qualified.
     * As a result, this must be awaited as rvalue (eg. using `co_await std::move(awaiter).wait()`).
     */
    auto wait() && -> internal::ImmediateAwaitable<internal::LowLevelEventAwaitable::Awaiter>;

  private:
    internal::LowLevelEventAwaitable internal_awaitable_;

    friend internal::LowLevelEventAwaitable internal::get_low_level_event_awaitable(EventAwaitable event_awaitable);
  };

} // namespace mrs_lib::coro

#endif // MRS_LIB_CORO_EVENT_HPP_
