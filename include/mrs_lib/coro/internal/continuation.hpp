#ifndef MRS_LIB_CORO_INTERNAL_CONTINUATION_HPP_
#define MRS_LIB_CORO_INTERNAL_CONTINUATION_HPP_


#include <coroutine>
#include <functional>
#include <stop_token>
#include <utility>

namespace mrs_lib::coro::internal
{

  /**
   * @brief Cast coroutine handle to the specified promise type.
   *
   * @tparam T Coroutine promise type to cast to.
   * @param handle Coroutine handle to cast.
   *
   * @warning Causes undefined behavior if the stored promise type is not the exact type we requested.
   */
  template <typename T>
  std::coroutine_handle<T> coroutine_handle_reinterpret_cast(std::coroutine_handle<> handle)
  {
    return std::coroutine_handle<T>::from_address(handle.address());
  }

  /**
   * @brief Type trait to obtain data necessary for creating CancellableContinuation.
   *
   * This type must be specialized for the correct promise type. The default template is undefined.
   *
   * Required members in specializations:
   * - `static CancellableContinuation release_continuation(std::coroutine_handle<T>)`
   * - `static std::stop_token get_token(std::coroutine_handle<T>)`
   */
  template <typename T>
  struct CancellableContinuationFor;

  template <>
  struct CancellableContinuationFor<void>
  {
  };

  /**
   * @brief Owning coroutine handle supporting cancellation.
   *
   * This class can be used to hold coroutine handle as owner. If it is
   * destroyed while holding a handle, it cancels the coroutine.
   *
   * Coroutine cancellation is performed in a way that should not exhaust stack
   * space.
   */
  class CancellableContinuation
  {
  private:
    template <typename T>
    using ContinuationGetterFptr = CancellableContinuation (*)(std::coroutine_handle<T>);
    template <typename T>
    using TokenGetterFptr = std::stop_token (*)(std::coroutine_handle<T>);

  public:
    /**
     * @brief Construct empty continuation.
     */
    CancellableContinuation() = default;

    /**
     * @brief Cancellable continuation cannot be constructed from type erased handle.
     */
    explicit CancellableContinuation(std::coroutine_handle<void>) = delete;

    /**
     * @brief Construct continuation that will own the passed handle.
     *
     * @param handle Handle to the continuation to store.
     */
    template <typename T>
    explicit CancellableContinuation(std::coroutine_handle<T> handle)
        : data_{
              .func = [](std::coroutine_handle<> handle) -> CancellableContinuation {
                using Trait = CancellableContinuationFor<T>;
                static_assert(std::same_as<decltype(&Trait::release_continuation), ContinuationGetterFptr<T>>,
                              "Wrong signature for CancellableContinuationFor<T>::release_continuation.");
                static_assert(std::same_as<decltype(&Trait::get_token), TokenGetterFptr<T>>, "Wrong signature for CancellableContinuationFor<T>::get_token.");
                return Trait::release_continuation(coroutine_handle_reinterpret_cast<T>(handle));
              },
              .handle = handle,
              .stop_token = CancellableContinuationFor<T>::get_token(coroutine_handle_reinterpret_cast<T>(handle)),
          }
    {
    }

    /**
     * @brief Destroy stored continuation if there is any.
     */
    ~CancellableContinuation()
    {
      cancel_and_destroy();
    }

    CancellableContinuation(const CancellableContinuation&) = delete;
    CancellableContinuation& operator=(const CancellableContinuation&) = delete;
    CancellableContinuation(CancellableContinuation&& other) noexcept : data_(std::exchange(other.data_, {}))
    {
    }
    CancellableContinuation& operator=(CancellableContinuation&& other) noexcept
    {
      std::ranges::swap(data_, other.data_);
      return *this;
    }

    /**
     * @brief Release ownership of the stored handle, giving it to the caller.
     *
     * The continuation will be empty after this call.
     */
    std::coroutine_handle<> release()
    {
      return std::exchange(data_.handle, nullptr);
    }

    /**
     * @brief Cancel the stored continuation, if there is any.
     *
     * The continuation will be empty after this call.
     */
    void cancel_and_destroy()
    {
      ContinuationData local_data = release_data();

      while (local_data.handle != nullptr)
      {
        CancellableContinuation continuation = std::invoke(local_data.func, local_data.handle);
        local_data.handle.destroy();
        local_data = continuation.release_data();
      }
    }

    /**
     * @brief Get stop token associated with the continuation.
     */
    std::stop_token get_token() const
    {
      return data_.stop_token;
    }

    /**
     * @brief Check if the current continuation is empty.
     */
    friend bool operator==(const CancellableContinuation& continuation, std::nullptr_t)
    {
      return continuation.data_.handle == nullptr;
    }

  private:
    /**
     * @brief Internal data of CancellableContinuation.
     */
    struct ContinuationData
    {
      ContinuationGetterFptr<void> func = nullptr;
      std::coroutine_handle<> handle = nullptr;
      std::stop_token stop_token{};
    };

    /**
     * @brief Release the data stored by the continuation. It will be empty after this call.
     */
    ContinuationData release_data()
    {
      return std::exchange(data_, {});
    }

    ContinuationData data_;
  };

} // namespace mrs_lib::coro::internal

#endif // MRS_LIB_CORO_INTERNAL_CONTINUATION_HPP_
