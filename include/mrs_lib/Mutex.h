#ifndef MUTEX_H
#define MUTEX_H

#include <mutex>

namespace mrs_lib
{

// getting a variable value protected by a mutex
// special case for a single variable and a mutex
template <class T>
T get_mutexed(T& var, std::mutex& mut) {
  std::scoped_lock lock(mut);

  return var;
}

// getting a tuple of variable values protected by a mutex
// the mutex is supposed to be the last argument
template <class... Args>
auto get_mutexed(Args&... args) {

  lockme(args...);
  std::tuple result = get_mutexed(args...);
  unlockme(args...);

  return result;
}

// getting a tuple of variables
// recursive, not the last one, just calling itself while peeling off the first argument
// It returns a new tuple created from the first peeled-off argument and the tuple, which
// return from the recursion.
template <class T, class... Args>
auto get_mutexed(T& first, Args&... args) {
  return std::tuple_cat(std::tuple(first), std::tuple(get_mutexed(args...)));
}

// the bottom of recursion
// Since the last argument is suposed to me the mutex, just returns an empty tuple.
template <class T>
auto get_mutexed([[maybe_unused]] T& var) {
  return std::make_tuple();
}

// locking a mutex, which is supposed to be the last function argument
// recursive, not the last one, just calling itself while peeling off the first argument
template <class T, class... Args>
void lockme([[maybe_unused]] T& first, Args&... args) {
  lockme(args...);
}

// locking a mutex, which is supposed to be the last function argument
// recursive, the LAST one, where the locking actually happens
template <class T>
void lockme(T& var) {
  var.lock();
}

// unlocking a mutex, which is supposed to be the last function argument
// recursive, not the last one, just calling itself while peeling off the first argument
template <class T>
void unlockme(T& var) {
  var.unlock();
}

// unlocking a mutex, which is supposed to be the last function argument
// recursive, the LAST one, where the unlocking actually happens
template <class T, class... Args>
void unlockme([[maybe_unused]] T& first, Args&... args) {
  unlockme(args...);
}

}  // namespace mrs_lib

#endif
