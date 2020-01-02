/**  \file
     \brief Defines helper routines for getting and setting variables under mutex locks
     \author Tomas Baca - tomas.baca@fel.cvut.cz
     \page handle mutex
 */
#ifndef MUTEX_H
#define MUTEX_H

#include <mutex>
#include <tuple>

namespace mrs_lib
{

/**
 * @brief thread-safe getter for values of variables (args)
 *
 * @tparam Args types of the variables
 * @param mut mutex which protects the variables
 * @param args variables to obtain the values from
 *
 * @return std::tuple of the values
 */
template <class... Args>
std::tuple<Args...> get_mutexed(std::mutex& mut, Args&... args) {

  std::scoped_lock lock(mut);

  std::tuple result = std::tuple(args...);

  return result;
}


/**
 * @brief thread-safe getter a value from a variable
 *
 * @tparam T type of the variable
 * @param mut mutex which protects the variable
 * @param arg variable to obtain the value from
 *
 * @return value of the variable
 */
template <class T>
T get_mutexed(std::mutex& mut, T& arg) {

  std::scoped_lock lock(mut);

  return arg;
}

/**
 * @brief base case of the variadic template for set_mutexed()
 *
 * @tparam T variable type
 * @param what value to set
 * @param where reference to be set
 */
template <class T>
void set_mutexed_impl(const T what, T& where) {

  where = what;
}

/**
 * @brief general case of the variadic template for set_mutexed()
 *
 * @tparam T type of the next variable to set
 * @tparam Args types of the rest of the variables
 * @param what value to set
 * @param where reference to be set
 * @param args the remaining arguments
 */
template <class T, class... Args>
void set_mutexed_impl(const T what, T& where, Args&... args) {

  where = what;

  set_mutexed_impl(args...);
}

/**
 * @brief thread-safe setter for a variable
 *
 * @tparam T type of the variable
 * @param mut mutex to be locked
 * @param what value to set
 * @param where reference to be set
 *
 * @return
 */
template <class T>
auto set_mutexed(std::mutex& mut, const T what, T& where) {

  std::scoped_lock lock(mut);

  where = what;

  return where;
}

/**
 * @brief thread-safe setter for multiple variables
 *
 * example:
 *   set_mutexed(my_mutex_, a, a_, b, b_, c, c_);
 *   where a, b, c are the values to be set
 *         a_, b_, c_ are the updated variables
 *
 * @tparam Args types of the variables
 * @param mut mutex to be locked
 * @param args
 *
 * @return alternating list of values that were just set
 */
template <class... Args>
auto set_mutexed(std::mutex& mut, Args&... args) {

  std::scoped_lock lock(mut);

  set_mutexed_impl(args...);

  return std::tuple(args...);
}

/**
 * @brief thread-safe setter for multiple variables
 *
 * example:
 *   set_mutexed(mu_mutex, std::tuple(a, b, c), std::forward_as_tuple(a_, b_, c_));
 *   where a, b, c are the values to be set
 *         a_, b_, c_ are the updated variables
 *
 * @tparam Args types of the variables
 * @param mut mutex to be locked
 * @param from std::tuple of the values
 * @param to std::tuple of reference to the variablaes
 *
 * @return
 */
template <class... Args>
auto set_mutexed(std::mutex& mut, const std::tuple<Args...> from, std::tuple<Args&...> to) {

  std::scoped_lock lock(mut);

  to = from;

  return to;
}

}  // namespace mrs_lib

#endif