/** @file */

#ifndef MRS_LIB_UTILITY_OWNING_MUTEX_HPP_
#define MRS_LIB_UTILITY_OWNING_MUTEX_HPP_


#include <concepts>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>


namespace mrs_lib
{

  /**
   * @tparam T The type of data to be protected by the mutex.
   * @tparam Mutex The mutex type to use (defaults to std::mutex).
   *
   * @brief A thread-safe wrapper for shared data with mutex protection.
   */
  template <typename T, typename Mutex = std::mutex>
    requires(std::same_as<T, std::remove_cvref_t<T>> && std::default_initializable<Mutex>)
  class OwningMutex
  {
  private:
    template <bool Mutable>
    class [[nodiscard]] OwningMutexGuard;

  public:
    /**
     * @brief Construct with default-initialized data.
     *
     * @note
     * This constructor is only available if the type of the contained
     * object is default constructible.
     */
    OwningMutex()
      requires(std::default_initializable<T>)
    = default;

    /**
     * @brief Construct by copying the data.
     *
     * @param val The value to copy into the OwningMutex.
     */
    OwningMutex(const T& val) : data_(val)
    {
    }

    /**
     * @brief Construct by moving the data.
     *
     * @param val The value to move into the OwningMutex.
     */
    OwningMutex(T&& val) : data_(std::move(val))
    {
    }

    /** @brief Default destructor. */
    ~OwningMutex() = default;

    /** @brief OwningMutex is immovable. */
    OwningMutex(const OwningMutex&) = delete;
    /** @brief OwningMutex is immovable. */
    OwningMutex& operator=(const OwningMutex&) = delete;
    /** @brief OwningMutex is immovable. */
    OwningMutex(OwningMutex&&) = delete;
    /** @brief OwningMutex is immovable. */
    OwningMutex& operator=(OwningMutex&&) = delete;

    /**
     * @brief Acquires a mutable guard for the OwningMutex.
     *
     * @return A mutable guard that allows modification of the data.
     */
    OwningMutexGuard<true> acquire()
    {
      return OwningMutexGuard<true>(*this);
    }

    /**
     * @brief Acquires a const guard for the OwningMutex.
     *
     * @return A const guard that allows read-only access to the data.
     */
    OwningMutexGuard<false> acquire() const
    {
      return OwningMutexGuard<false>(*this);
    }

    /**
     * @brief Loads the current value of the data.
     *
     * @return The current value of the data.
     */
    [[nodiscard]] T load() const
    {
      std::lock_guard lock(mutex_);
      return data_;
    }

    /**
     * @brief Stores a new value into the data.
     *
     * @param val The value to store.
     *
     * @note
     * This method requires the type of the contained class to be noexcept
     * move constructible.
     */
    void store(T val)
      requires(std::is_nothrow_move_assignable_v<T>)
    {
      std::lock_guard lock(mutex_);
      data_ = std::move(val);
    }

  private:
    mutable Mutex mutex_;
    T data_;
  };

  /**
   * @tparam Mutable Whether the guard can modify the data (true) or only read (false).
   * @tparam T The type of data to be protected by the mutex.
   * @tparam Mutex The mutex type.
   *
   * @brief A mutex guard for OwningMutex to ensure thread-safe access.
   *
   * The OwningMutexGuard class provides RAII-style mutex locking and unlocking.
   * It ensures that the mutex is locked when the guard is constructed and unlocked
   * when the guard is destroyed.
   *
   * @note
   * To obtain instance of this class, use OwningMutex::acquire.
   */
  template <typename T, typename Mutex>
    requires(std::same_as<T, std::remove_cvref_t<T>> && std::default_initializable<Mutex>)
  template <bool Mutable>
  class OwningMutex<T, Mutex>::OwningMutexGuard
  {
  public:
    /** @brief Default destructor. */
    ~OwningMutexGuard() = default;

    /** @brief OwningMutexGuard is immovable. */
    OwningMutexGuard(const OwningMutexGuard&) = delete;
    /** @brief OwningMutexGuard is immovable. */
    OwningMutexGuard& operator=(const OwningMutexGuard&) = delete;
    /** @brief OwningMutexGuard is immovable. */
    OwningMutexGuard(OwningMutexGuard&&) = delete;
    /** @brief OwningMutexGuard is immovable. */
    OwningMutexGuard& operator=(OwningMutexGuard&&) = delete;

    /**
     * @brief Dereference operator for mutable access.
     *
     * @return Reference to the data.
     *
     * @note
     * This method is only available if Mutable is true.
     */
    T& operator*() &
      requires(Mutable)
    {
      return parent_.data_;
    }

    /**
     * @brief Dereference operator for const access.
     *
     * @return Reference to the data.
     */
    const T& operator*() const&
    {
      return parent_.data_;
    }

    /** @brief Rvalue overloads are deleted to prevent accidental unlocking. */
    T& operator*() && = delete;
    /** @brief Rvalue overloads are deleted to prevent accidental unlocking. */
    const T& operator*() const&& = delete;

    /**
     * @brief Arrow operator for mutable access.
     *
     * @return Pointer to the data.
     *
     * @note
     * This method is only available if Mutable is true.
     */
    T* operator->() &
      requires(Mutable)
    {
      return std::addressof(parent_.data_);
    }

    /**
     * @brief Arrow operator for const access.
     *
     * @return Pointer to the data.
     */
    const T* operator->() const&
    {
      return std::addressof(parent_.data_);
    }

    /** @brief Rvalue overloads are deleted to prevent accidental unlocking. */
    T* operator->() && = delete;
    /** @brief Rvalue overloads are deleted to prevent accidental unlocking. */
    const T* operator->() const&& = delete;

  private:
    using RefType = std::conditional_t<Mutable, OwningMutex&, const OwningMutex&>;

    /**
     * @brief Construct the object guarding the selected OwningMutex.
     *
     * @param parent Reference to the OwningMutex instance to guard.
     */
    explicit OwningMutexGuard(RefType parent) : lock_(parent.mutex_), parent_(parent)
    {
    }

    std::lock_guard<Mutex> lock_;
    RefType parent_;

    friend class OwningMutex;
  };

} // namespace mrs_lib


#endif // MRS_LIB_UTILITY_OWNING_MUTEX_HPP_
