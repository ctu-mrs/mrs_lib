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

int sign(double a) {

  if (a > 0) {
    return 1;
  }

  if (a < 0) {
    return -1;
  }

  return 0;
}

}  // namespace mrs_lib

#endif
