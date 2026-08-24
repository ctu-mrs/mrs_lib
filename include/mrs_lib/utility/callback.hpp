/** @file */

#ifndef MRS_LIB_UTILITY_CALLBACK_HPP_
#define MRS_LIB_UTILITY_CALLBACK_HPP_

#include <atomic>
#include <concepts>
#include <functional>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <type_traits>
#include <utility>
#include <variant>

#include "mrs_lib/coro/task.hpp"
#include "mrs_lib/utility/meta.hpp"
#include "mrs_lib/utility/scope_cleanup.hpp"

namespace mrs_lib
{

  /**
   * @brief Namespace for tag types used in CoroCallback.
   */
  namespace coro_callback_tags
  {
    /**
     * @brief Policy tag for reentrant CoroCallback.
     *
     * This policy tells the callback to run every time it is called.
     */
    class Reentrant
    {
    public:
      explicit Reentrant() = default;
    };

    /**
     * @brief Policy tag for CoroCallback ignores new callbacks, when another is already running.
     * @tparam CallbackRetType Return type of the callback.
     *
     * This policy tells the callback to run only if it is not already running.
     * If this callback is already running, the new callback is cancelled.
     * All copies of the callback share the same state and only one can be run at the same time.
     * However, different callbacks created by the main constructor can run at the same time.
     *
     * Since invoking every callback must return a value according to the return
     * type fo the callback, if it not launched, the return value must be obtained
     * somewhere else. For this purpose, this policy stores a factory function that
     * creates the return value for cancelled callbacks.
     *
     * @note
     * If you want to return a default constructed value, you can use the CancelNewDefault tag instead.
     *
     * @warning
     * When a callback is canceled, you will lose the event that triggered it.
     * This means that when using this as e.g. a subscription callback, you might miss some messages.
     */
    template <typename CallbackRetType = void>
    class CancelNew
    {
    public:
      /**
       * @brief Constructor accepting factory function.
       * @param default_factory Factory function for return value of cancelled callbacks.
       */
      explicit CancelNew(std::function<CallbackRetType()> default_factory) : default_factory_(std::move(default_factory))
      {
      }

      /**
       * @brief Create a return value for a cancelled callback.
       * @return Value created by the stored factory function.
       */
      CallbackRetType create_value() const
      {
        return default_factory_();
      }

    private:
      std::function<CallbackRetType()> default_factory_;
    };

    /**
     * @brief Void specialization of CancelNew.
     */
    template <>
    class CancelNew<void>
    {
    public:
      /**
       * @brief Default constructor.
       */
      explicit CancelNew() = default;

      /**
       * @brief Returns void.
       */
      void create_value() const
      {
      }
    };

    /**
     * @brief Deduction guide to deduce CancelNew type from factory function.
     */
    template <typename F>
    CancelNew(F) -> CancelNew<std::invoke_result_t<F>>;

    /**
     * @brief Policy tag for CoroCallback ignores new callbacks, when another is already running.
     *
     * Can be used instead of CancelNew for any default constructible type or void.
     *
     * @see CancelNew
     */
    class CancelNewDefault
    {
    public:
      /**
       * @brief Default constructor.
       */
      explicit CancelNewDefault() = default;

      /**
       * @brief Conversion operator to create CancelNew<void>.
       */
      operator CancelNew<void>()
      {
        return CancelNew<>{};
      }

      /**
       * @brief Conversion operator to create CancelNew<T> for any default constructible T.
       *
       * The created CancelNew policy will contain factory creating default
       * constructed values of type T.
       */
      template <typename T>
        requires std::default_initializable<T>
      operator CancelNew<T>()
      {
        return CancelNew<T>([] { return T{}; });
      }
    };

  } // namespace coro_callback_tags

  /**
   * @brief Variant of CoroCallback reentrancy policies.
   *
   * @tparam CallbackRetType Return type of the callback function.
   *
   */
  template <typename CallbackRetType>
  using CoroReentrantPolicyVariant = std::variant<coro_callback_tags::Reentrant, coro_callback_tags::CancelNew<CallbackRetType>>;

  /**
   * @brief Base template for CoroCallback.
   *
   * @see CoroCallback<CallbackRetType(CallbackArgs...)>
   */
  template <typename...>
  class CoroCallback
  {
    static_assert(false, "Base template selected for CoroCallback");
  };


  /**
   * @brief Wrapper for safe binding of coroutines to use as callbacks.
   *
   * @tparam CallbackRetType Return type of the callback coroutine.
   * @tparam CallbackArgs Argument types of the callback coroutine.
   *
   * This wrapper is used to bind arguments to coroutines and pass them to other
   * interfaces that will use this as a callback.
   *
   * Since coroutines do not work well with mutually exclusive callback group,
   * this wrapper adds some functionality to configure behavior of reentrant
   * callbacks. The behavior is chosen using policy argument passed in the
   * constructor.
   */
  template <typename CallbackRetType, typename... CallbackArgs>
  class CoroCallback<CallbackRetType(CallbackArgs...)>
  {
  private:
    using FunctionSignature = coro::Task<CallbackRetType>(CallbackArgs...);

  public:
    /**
     * @brief Constructor for binding coroutine callbacks.
     *
     * @tparam F Callback function type.
     * @tparam BoundArgs Types of arguments to bind to the callback function.
     * @param reentrant_policy Reentrancy policy for the callback.
     * @param callback_ptr Callback function pointer (or member function pointer).
     * @param args Arguments to bind to the callback function.
     *
     * Constructs the callback by binding @p args to the front parameters of callback.
     *
     * The constructed callback will have reentrant behavior according to the specified policy.
     *
     * @see CoroReentrantPolicyVariant
     */
    template <typename F, typename... BoundArgs>
      requires((std::is_member_function_pointer_v<F> || (std::is_pointer_v<F> && std::is_function_v<std::remove_pointer_t<F>>))
               && std::invocable<F, const std::decay_t<BoundArgs>&..., CallbackArgs...>
               && std::same_as<coro::Task<CallbackRetType>, std::invoke_result_t<F, const std::decay_t<BoundArgs>&..., CallbackArgs...>>)
    explicit CoroCallback(CoroReentrantPolicyVariant<CallbackRetType> reentrant_policy, F callback_ptr, BoundArgs&&... args)
        : func_(bind_callback(std::move(reentrant_policy), callback_ptr, std::forward<BoundArgs>(args)...))
    {
    }

    /**
     * @brief Executes the callback.
     *
     * @param args Arguments to pass to the callback.
     * @return The result of the callback execution.
     */
    coro::Task<CallbackRetType> operator()(CallbackArgs... args) const
    {
      co_return co_await std::invoke(func_, std::forward<CallbackArgs>(args)...);
    }

  private:
    /**
     * @brief Wraps a reentrant callback function.
     *
     * @tparam F Callback function type.
     * @param - Reentrant tag.
     * @param func Bound callback function.
     * @return The bound function.
     */
    template <typename F>
    static std::function<FunctionSignature> wrap_callback(coro_callback_tags::Reentrant, F&& func)
    {
      return std::forward<F>(func);
    }

    /**
     * @brief Wraps a callback function with cancellation policy.
     *
     * @tparam F Callback function type.
     * @param cancel_new_policy Cancellation policy.
     * @param func Bound callback function.
     * @return Function wrapped with the cancellation system.
     */
    template <typename F>
    static std::function<FunctionSignature> wrap_callback(coro_callback_tags::CancelNew<CallbackRetType> cancel_new_policy, F&& func)
    {
      return [running = std::make_shared<std::atomic_bool>(false), cancel_new_policy = std::move(cancel_new_policy),
              func = std::forward<F>(func)](CallbackArgs... args) -> coro::Task<CallbackRetType> {
        bool was_running = running->exchange(true);
        // !was_running <=> we set the state to running => we should start the callback
        if (!was_running)
        {
          ScopeCleanup cleanup([&] { running->store(false); });
          co_return co_await func(std::forward<CallbackArgs>(args)...);
        } else
        {
          co_return cancel_new_policy.create_value();
        }
      };
    }

    /**
     * @brief Creates the internal bound callback.
     *
     * @tparam F Callback function type.
     * @tparam BoundArgs The arguments to bind to the callback function.
     * @param reentrant_policy Reentrancy policy for the callback.
     * @param callback_ptr Callback function.
     * @param args Arguments to bind to the callback function.
     * @return Bound callback with the specified reentrancy policy.
     */
    template <typename F, typename... BoundArgs>
    static std::function<FunctionSignature> bind_callback(CoroReentrantPolicyVariant<CallbackRetType> reentrant_policy, F callback_ptr, BoundArgs&&... args)
    {
      if (callback_ptr == nullptr)
      {
        throw std::logic_error("Cannot construct CoroCallback from nullptr.");
      }

      auto bound_callback = [callback = std::move(callback_ptr),
                             ... pre_args = std::forward<BoundArgs>(args)](CallbackArgs... args) -> coro::Task<CallbackRetType> {
        co_return co_await std::invoke(callback, pre_args..., std::forward<CallbackArgs>(args)...);
      };

      return std::visit(
          [&](auto&& reentrant_policy) -> std::function<FunctionSignature> {
            return wrap_callback(std::forward<decltype(reentrant_policy)>(reentrant_policy), std::move(bound_callback));
          },
          reentrant_policy);
    }

    std::function<FunctionSignature> func_;
  };

  namespace internal
  {

    /**
     * @brief Type trait to determine the signature of a callback function.
     */
    template <typename...>
    struct CallbackSignature;

    /**
     * @brief Type trait to determine the signature of a callback function.
     * @tparam Ret Return type of the callback.
     * @tparam Params Theparameter types of the callback.
     * @tparam Args The argument types to bind to the callback.
     */
    template <typename Ret, typename... Params, typename... Args>
    struct CallbackSignature<Ret, meta::TypeList<Params...>, meta::TypeList<Args...>>
    {
    private:
      template <typename... InnerParams>
      using FuncSignature = Ret(InnerParams...);

    public:
      using type = meta::ApplyT<FuncSignature, meta::DropT<meta::TypeList<Params...>, sizeof...(Args)>>;
    };

    /**
     * @brief Helper type alias for CallbackSignature<Ret, Params, Args>::type.
     * @tparam Ret Return type of the callback.
     * @tparam Params Parameter types of the callback.
     * @tparam Args Argument types to bind to the callback.
     */
    template <typename Ret, typename Params, typename Args>
    using CallbackSignatureT = CallbackSignature<Ret, Params, Args>::type;

  } // namespace internal

  /**
   * @brief Deduction guide for free functions.
   */
  template <typename R, typename... Params, typename... Args, bool Noexcept>
  CoroCallback(auto, mrs_lib::Task<R> (*)(Params...) noexcept(Noexcept), Args&&...) //
      -> CoroCallback<internal::CallbackSignatureT<R, meta::TypeList<Params...>, meta::TypeList<Args...>>>;

  /**
   * @brief Deduction guide for non const member functions.
   */
  template <typename C1, typename C2, typename R, typename... Params, typename... Args, bool Noexcept>
  CoroCallback(auto, mrs_lib::Task<R> (C1::*)(Params...) noexcept(Noexcept), C2, Args&&...) //
      -> CoroCallback<internal::CallbackSignatureT<R, meta::TypeList<Params...>, meta::TypeList<Args...>>>;

  /**
   * @brief Deduction guide for const member functions.
   */
  template <typename C1, typename C2, typename R, typename... Params, typename... Args, bool Noexcept>
  CoroCallback(auto, mrs_lib::Task<R> (C1::*)(Params...) const noexcept(Noexcept), C2, Args&&...) //
      -> CoroCallback<internal::CallbackSignatureT<R, meta::TypeList<Params...>, meta::TypeList<Args...>>>;

} // namespace mrs_lib

#endif // MRS_LIB_UTILITY_CALLBACK_HPP_
