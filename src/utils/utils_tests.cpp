#include <mrs_lib/Utils.h>
#include <iostream>

int main()
{
  std::vector<int> vec = {1, 2, 666};
  std::string str = mrs_lib::containerToString(std::begin(vec)+1, std::end(vec), ";");
  std::string str2 = mrs_lib::containerToString(vec, ", ");
  std::cout << str << std::endl;
  std::cout << str2 << std::endl;
}

