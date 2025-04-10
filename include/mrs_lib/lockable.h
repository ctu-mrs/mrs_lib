#pragma once

#include <mutex>

namespace mrs_lib
{
  template <typename LockedVarT, typename MutexT>
  class Unlocker;

  /**
   * \brief Convenience class for safe and expressive management of mutexed variables.
   *
   * In the typical use-case, instantiate this class with a struct holding all variables
   * that you want locked together.
   *
   * To access these variables, unlock the object with the specialized RAII-style object
   * Unlocker (see below in this file) - similar to how you'd use an std::scoped_lock.
   * Or you can use the familiar set_mutexed() and get_mutexed() methods.
   *
   */
  template <typename LockedVarT, typename MutexT = std::mutex>
  class Lockable
  {
    MutexT mtx;
    LockedVarT locked_var;

    friend Unlocker<LockedVarT, MutexT>;

  public:
    Lockable() = default;

    Lockable(const LockedVarT& init_val)
      : locked_var(init_val)
    {}

    Lockable(const LockedVarT&& init_val)
      : locked_var(init_val)
    {}

    LockedVarT& unsafe_access()
    {
      return locked_var;
    }

    void set_mutexed(const LockedVarT& val)
    {
      std::scoped_lock lock(mtx);
      locked_var = val;
    }

    LockedVarT get_mutexed()
    {
      std::scoped_lock lock(mtx);
      return locked_var;
    }

    MutexT& mutex()
    {
      return mtx;
    }
  };

  /**
   * \brief Convenience class for RAII-style access to a varible within a Lockable object.
   *
   * Locks the internal mutex of the Lockable on construction, unlocks it on deconstruction,
   * providing thread-safe pointer-like access to the variable within using operator*() and
   * operator->().
   *
   */
  template <typename LockedVarT, typename MutexT>
  class Unlocker
  {
  public:
    Unlocker(Lockable<LockedVarT, MutexT>& lockable)
      : lock(lockable.mtx), lockable(lockable)
    {}

    LockedVarT& operator*()
    {
      return lockable.locked_var;
    }

    LockedVarT* operator->()
    {
      return &(lockable.locked_var);
    }

  private:
    std::scoped_lock<MutexT> lock;
    Lockable<LockedVarT>& lockable;
  };

}
