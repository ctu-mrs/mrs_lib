#ifndef UTILS_H
#define UTILS_H

namespace mrs_lib
{

// | - context solution for automatically unsetting a variable  |
class ScopeUnset {

public:
  ScopeUnset();
  ScopeUnset(bool &in);
  ~ScopeUnset();

private:
  bool &variable;
};

}  // namespace mrs_lib

#endif
