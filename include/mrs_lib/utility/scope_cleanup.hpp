#ifndef MRS_LIB_UTILITY_SCOPE_CLEANUP_HPP_
#define MRS_LIB_UTILITY_SCOPE_CLEANUP_HPP_


#include <functional>
#include <utility>


namespace mrs_lib
{

  /**
   * @brief Utility class for running cleanup at the end of a scope.
   */
  template <typename T>
  class ScopeCleanup
  {
  public:
    /**
     * @brief Create a scope cleanup that runs the specified callable when destroyed.
     *
     * @note The object must be stored in a variable, otherwise, it will be destroyed immediately.
     */
    explicit ScopeCleanup(T f) : cleanup_func_(std::move(f))
    {
    }

    ~ScopeCleanup()
    {
      if (enabled_)
      {
        std::invoke(cleanup_func_);
      }
    }

    ScopeCleanup(const ScopeCleanup&) = delete;
    ScopeCleanup(ScopeCleanup&&) = delete;
    ScopeCleanup& operator=(const ScopeCleanup&) = delete;
    ScopeCleanup& operator=(ScopeCleanup&&) = delete;

    /**
     * @brief Disable running of the callback, when the cleanup object is destroyed.
     */
    void cancel()
    {
      enabled_ = false;
    }

  private:
    T cleanup_func_;
    bool enabled_ = true;
  };

} // namespace mrs_lib


#endif // MRS_LIB_UTILITY_SCOPE_CLEANUP_HPP_
