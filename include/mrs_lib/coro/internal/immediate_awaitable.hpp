#ifndef MRS_LIB_CORO_INTERNAL_IMMEDIATE_AWAITABLE_HPP_
#define MRS_LIB_CORO_INTERNAL_IMMEDIATE_AWAITABLE_HPP_


#include <utility>


namespace mrs_lib::coro::internal
{

  /**
   * @brief Helper class to force awaiting a result in the current expression.
   */
  template <typename Awaiter>
  class [[nodiscard("The result must be awaited, otherwise, it may do NOTHING!!!")]] ImmediateAwaitable
  {

  public:
    ImmediateAwaitable()
      requires std::default_initializable<Awaiter>
    = default;
    ImmediateAwaitable(Awaiter awaiter) : awaiter_(std::move(awaiter))
    {
    }

    ~ImmediateAwaitable() = default;

    ImmediateAwaitable(const ImmediateAwaitable&) = delete;
    ImmediateAwaitable(ImmediateAwaitable&&) = delete;
    ImmediateAwaitable& operator=(const ImmediateAwaitable&) = delete;
    ImmediateAwaitable& operator=(ImmediateAwaitable&&) = delete;

    friend Awaiter operator co_await(ImmediateAwaitable awaitable)
    {
      return std::move(awaitable).awaiter_;
    }

  private:
    Awaiter awaiter_{};
  };

} // namespace mrs_lib::coro::internal


#endif // MRS_LIB_CORO_INTERNAL_IMMEDIATE_AWAITABLE_HPP_
