// clang: MatousFormat
#ifndef UTILS_H
#define UTILS_H

#include <iterator>

namespace mrs_lib
{

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
