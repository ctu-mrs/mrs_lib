#ifndef MRS_LIB_CORO_INTERNAL_RESULT_STORAGE_HPP_
#define MRS_LIB_CORO_INTERNAL_RESULT_STORAGE_HPP_


#include <cstddef>
#include <exception>
#include <variant>


namespace mrs_lib::coro::internal
{

  /**
   * @brief A variant-like class for storing the result of non-void task.
   */
  template <typename T>
  class ResultStorage
  {
  private:
    // Not enum class to allow usage in functions like std::get
    enum State : size_t
    {
      empty = 0,
      value = 1,
      exception = 2,
    };

  public:
    constexpr ResultStorage() noexcept : data_()
    {
    }

    /**
     * @brief Store result of task.
     *
     * This can only be called once and not if set_exception was called.
     */
    constexpr void set_value(T&& val) noexcept(std::is_nothrow_move_constructible_v<T>)
    {
      assert(data_.index() == State::empty);
      data_.template emplace<State::value>(std::move(val));
    }

    /**
     * @brief Store exception into the result.
     *
     * This can only ve called once and not if set_value was called (unless it failed with exception).
     */
    void set_exception(std::exception_ptr eptr) noexcept
    {
      assert(data_.index() == State::empty || data_.valueless_by_exception());
      data_.template emplace<State::exception>(std::move(eptr));
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
      size_t state = data_.index();
      if (state == State::exception)
      {
        std::rethrow_exception(std::get<State::exception>(data_));
      }
      assert(state == State::value);
      return std::get<State::value>(std::move(data_));
    }

  private:
    std::variant<std::monostate, T, std::exception_ptr> data_;
  };

} // namespace mrs_lib::coro::internal


#endif // MRS_LIB_CORO_INTERNAL_RESULT_STORAGE_HPP_
