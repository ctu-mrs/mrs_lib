#include <mrs_lib/utils.h>
#include <iostream>
#include <boost/array.hpp>

int main() {
  std::vector<int> vec = {1, 2, 666};
  std::string      str = mrs_lib::containerToString(std::begin(vec) + 1, std::end(vec), ";");

  boost::array<int, 3> arr  = {6, 6, 6};
  std::string          str2 = mrs_lib::containerToString(arr, ", ");
  std::cout << str << std::endl;
  std::cout << str2 << std::endl;
}

