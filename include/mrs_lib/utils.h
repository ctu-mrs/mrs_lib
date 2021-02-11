// clang: MatousFormat
/**  \file
     \brief Defines various general utility functions.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \author Viktor Walter - walver.viktor@fel.cvut.cz
     \author Tomáš Báča - baca.tomas@fel.cvut.cz
 */
#ifndef UTILS_H
#define UTILS_H

#include <iterator>
#include <vector>
#include <sstream>
#include <atomic>

namespace mrs_lib
{
  /* containerToString() function //{ */

  /**
   * \brief Convenience function for converting container ranges to strings (e.g. for printing).
   *
   * \param begin        first element of the container that will be converted to \p std::string.
   * \param end          one-after-the-last element of the container that will be converted to \p std::string.
   * \param delimiter    will be used to separate the elements in the output.
   * \return             elements of the container from \p begin to \p end (excluding), converted to string and separated by \p delimiter.
   *
   */
  template <typename Iterator>
  std::string containerToString(const Iterator begin, const Iterator end, const std::string& delimiter = ", ")
  {
    bool first = true;
    std::ostringstream output;
    for (Iterator it = begin; it != end; it++)
    {
      if (!first)
        output << delimiter;
      output << *it;
      first = false;
    }
    return output.str();
  }

  /**
   * \brief Convenience function for converting container ranges to strings (e.g. for printing).
   *
   * \param begin        first element of the container that will be converted to \p std::string.
   * \param end          one-after-the-last element of the container that will be converted to \p std::string.
   * \param delimiter    will be used to separate the elements in the output.
   * \return             elements of the container from \p begin to \p end (excluding), converted to string and separated by \p delimiter.
   *
   */
  template <typename Iterator>
  std::string containerToString(const Iterator begin, const Iterator end, const char* delimiter)
  {
    return containerToString(begin, end, std::string(delimiter));
  }

  /**
   * \brief Convenience function for converting containers to strings (e.g. for printing).
   *
   * \param cont         the container that will be converted to \p std::string.
   * \param delimiter    will be used to separate the elements in the output.
   * \return             elements of the container from \p begin to \p end (excluding), converted to string and separated by \p delimiter.
   *
   */
  template <typename Container>
  std::string containerToString(const Container& cont, const std::string& delimiter = ", ")
  {
    return containerToString(std::begin(cont), std::end(cont), delimiter);
  }

  /**
   * \brief Convenience function for converting containers to strings (e.g. for printing).
   *
   * \param cont         the container that will be converted to \p std::string.
   * \param delimiter    will be used to separate the elements in the output.
   * \return             elements of the container from \p begin to \p end (excluding), converted to string and separated by \p delimiter.
   *
   */
  template <typename Container>
  std::string containerToString(const Container& cont, const char* delimiter = ", ")
  {
    return containerToString(std::begin(cont), std::end(cont), std::string(delimiter));
  }

  //}

  /* remove_const() function //{ */

  /**
   * \brief Convenience class for removing const-ness from a container iterator.
   *
   * \param it    the iterator from which const-ness should be removed.
   * \param cont  the corresponding container of the iterator.
   * \return      a non-const iterator, pointing to the same element as \p it.
   *
   */
  template <typename T>
  typename T::iterator remove_const(const typename T::const_iterator& it, T& cont)
  {
    typename T::iterator ret = cont.begin();
    std::advance(ret, std::distance((typename T::const_iterator)ret, it));
    return ret;
  }

  //}

  /**
   * \brief Convenience class for automatically setting and unsetting an atomic boolean based on the object's scope.
   * Useful e.g. for indicating whether a thread is running or not.
   *
   */
  class AtomicScopeFlag
  {

  public:
  /**
   * \brief The constructor. Sets the flag \p in to \p true.
   *
   * \param in  The flag to be set on construction of this object and reset (set to \p false) on its destruction.
   *
   */
    AtomicScopeFlag(std::atomic<bool>& in);
  /**
   * \brief The destructor. Resets the variable given in the constructor to \p false.
   *
   */
    ~AtomicScopeFlag();

  private:
    std::atomic<bool>& variable;
  };

  // branchless, templated, more efficient version of the signum function
  // taken from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
  template <typename T>
  int signum(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

}  // namespace mrs_lib

#endif
