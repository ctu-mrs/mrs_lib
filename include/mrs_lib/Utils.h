#ifndef UTILS_H
#define UTILS_H

namespace mrs_lib
{

// | - context solution for automatically unsetting a variable  |
class ContextUnset {

public:
  ContextUnset();
  ContextUnset(bool &in);
  ~ContextUnset();

private:
  bool &variable;
};

}  // namespace mrs_lib

#endif
