// clang: MatousFormat
#ifndef UTILS_H
#define UTILS_H

#include <iterator>
#include <vector>
#include <iostream>
#include <boost/array.hpp>

namespace mrs_lib
{
  //This could probaby be polymorphised more elegantly by somehow taking general input with iterator, but I can't be bothered to research this
  template<typename T, typename A>
  std::string vectorToString(std::vector<T,A> const& input, const std::string& delimiter){
    bool first = true;
    std::ostringstream output;
    for (auto e : input){
      if (!first)
      output << delimiter;
      output << e;
        first = false;
    }
    return output.str();
  }
  template<typename T, typename A>
  std::string vectorToString(const std::vector<T,A>& input, const char* delimiter){
    return vectorToString(input, std::string(delimiter));
  }
  template< class T, std::size_t N >
  std::string vectorToString(const boost::array<T,N>& input, const char* delimiter){
    return vectorToString(std::vector<T>(input.begin(), input.end()), std::string(delimiter));
  }

  /* remove_cons() function //{ */
  // TODO what does this do?

  template <typename T>
  typename T::iterator remove_const(const typename T::const_iterator& it, T& cont)
  {
    typename T::iterator ret = cont.begin();
    std::advance(ret, std::distance((typename T::const_iterator)ret, it));
    return ret;
  }

  //}

  // | - context solution for automatically unsetting a variable  |
  class ScopeUnset
  {

  public:
    ScopeUnset();
    ScopeUnset(bool& in);
    ~ScopeUnset();

  private:
    bool& variable;
  };

  // branchless, templated, more efficient version of sign
  // taken from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
  template <typename T>
  int sign(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  /* int sign(double a) { */

  /*   if (a > 0) { */
  /*     return 1; */
  /*   } */

  /*   if (a < 0) { */
  /*     return -1; */
  /*   } */

  /*   return 0; */
  /* } */

}  // namespace mrs_lib

#endif
