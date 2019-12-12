#ifndef MUTEX_H
#define MUTEX_H

#include <mutex>
#include <tuple>

namespace mrs_lib
{

template <class... Args>
auto get_mutexed(std::mutex& mut, Args&... args) {

  std::scoped_lock lock(mut);

  std::tuple result = std::tuple(args...);

  return result;
}

template <class T>
T get_mutexed(std::mutex& mut, T& arg) {

  std::scoped_lock lock(mut);

  return arg;
}

template <class T, class... Args>
auto set_mutexed_even(T& first, Args&... args);
template <class T>
auto set_mutexed_odd(T& first);
template <class T, class... Args>
auto set_mutexed_odd(T& first, Args&... args);

template <class T>
auto set_mutexed_odd(T& first) {

  return first;
}

template <class T, class... Args>
auto set_mutexed_odd(T& first, Args&... args) {

  set_mutexed_even(args...);
  return first;
}

template <class T, class... Args>
auto set_mutexed_even(T& first, Args&... args) {

  first = set_mutexed_odd(args...);
}

template <class... Args>
void set_mutexed(std::mutex& mut, Args&... args) {

  std::scoped_lock lock(mut);

  set_mutexed_even(args...);
}

template <class... Args>
void set_mutexed(std::mutex& mut, std::tuple<Args&...> to, std::tuple<Args&...> from) {

  std::scoped_lock lock(mut);

  to = from;
}

}  // namespace mrs_lib

#endif
