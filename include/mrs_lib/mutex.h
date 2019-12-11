#ifndef MUTEX_H
#define MUTEX_H

#include <mutex>

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

}  // namespace mrs_lib

#endif
