/** @file */

#ifndef MRS_LIB_EXPERIMENTAL_UTILITY_PIMPL_HPP_
#define MRS_LIB_EXPERIMENTAL_UTILITY_PIMPL_HPP_


#include <cassert>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>


namespace mrs_lib
{

  /**
   * @tparam Interface The interface / class type that the Pimpl will hold.
   * @tparam StorageType The type of value used to store for the implementation (defaults to `std::unique_ptr<Interface>`).
   *
   * @brief Owning pointer wrapper with move only value semantics. Usefull for implementing the pointer to implementation (PIMPL) pattern.
   *
   * Contains the specified value using the StorageType.
   * Compared to std::*_ptr, this class uses deep const semantics and
   * is not nullable (with the exception of the moved from state in which it
   * should not be used).
   *
   * When implementing the PIMPL pattern for your own classes, the default
   * pointer type should be correct.
   *
   * This class can also be used to wrap non nullable ros interfaces
   * (eg. `rclcpp::Publisher`), which are provided as std::shared_ptr.
   * In those cases you can set StorageType to std::shared_ptr
   * (eg. for the publisher: `mrs_lib::Pimpl<rclcpp::Publisher, std::shared_ptr<rclcpp::Publisher>>`).
   * The class will keep the same semantics, but will use different storage.
   */
  template <typename Interface, typename StorageType = std::unique_ptr<Interface>>
  class Pimpl
  {
  public:
    /**
     * @brief Deleted constructor to prevent construction from nullptr.
     */
    explicit Pimpl(std::nullptr_t) = delete;

    /**
     * @brief Construct Pimpl from a pointer to the object.
     *
     * @param impl The pointer to the implementation object.
     * @throws std::logic_error if impl is `nullptr`.
     */
    explicit Pimpl(StorageType impl) : impl_(std::move(impl))
    {
      if (impl_ == nullptr)
      {
        throw std::logic_error("Cannot construct Pimpl from nullptr.");
      }
    }

    /**
     * @brief Construct the wrapped object in place.
     *
     * @tparam T The concrete type of the object to construct.
     * @tparam Args Types of arguments passed to the inner object's constructor.
     * @param - Tag type to distinguish between overloads and select the concrete type.
     * @param args Arguments to construct the inner object with.
     */
    template <typename T, typename... Args>
    explicit Pimpl(std::in_place_type_t<T>, Args&&... args) : impl_(std::make_unique<T>(std::forward<Args>(args)...))
    {
    }

    /** @brief Default destructor. */
    ~Pimpl() = default;

    /** @brief Pimpl is not copyable. */
    Pimpl(const Pimpl&) = delete;
    /** @brief Pimpl is not copyable. */
    Pimpl& operator=(const Pimpl&) = delete;
    /** @brief Default move constructor. */
    Pimpl(Pimpl&&) = default;
    /** @brief Default move assignment operator. */
    Pimpl& operator=(Pimpl&&) = default;

    /**
     * @brief Dereference operator for non-const access.
     *
     * @return Reference to the underlying implementation object.
     */
    [[nodiscard]] Interface& operator*() noexcept
    {
      assert(impl_ != nullptr);
      return *impl_;
    }

    /**
     * @brief Dereference operator for const access.
     *
     * @return Reference to the underlying implementation object.
     */
    [[nodiscard]] const Interface& operator*() const noexcept
    {
      assert(impl_ != nullptr);
      return *impl_;
    }

    /**
     * @brief Arrow operator for non-const access.
     *
     * @return Pointer to the underlying implementation object.
     */
    [[nodiscard]] Interface* operator->() noexcept
    {
      assert(impl_ != nullptr);
      return impl_.get();
    }

    /**
     * @brief Arrow operator for const access.
     *
     * @return Pointer to the underlying implementation object.
     */
    [[nodiscard]] const Interface* operator->() const noexcept
    {
      assert(impl_ != nullptr);
      return impl_.get();
    }

    /**
     * @brief Checks if the object is in moved from state.
     *
     * Can be used on moved from object.
     *
     * @note
     * This function should not be needed in most cases.
     * You should not keep moved from objects alive for long periods of time.
     */
    [[nodiscard]] bool valueless_after_move() const noexcept
    {
      return impl_ == nullptr;
    }

  private:
    StorageType impl_;
  };

} // namespace mrs_lib


#endif // MRS_LIB_EXPERIMENTAL_UTILITY_PIMPL_HPP_
