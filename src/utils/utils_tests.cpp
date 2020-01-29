#include <mrs_lib/Utils.h>
#include <iostream>

int main()
{
  std::vector<int> vec = {1, 2, 666};
  std::string str = mrs_lib::containerToString(std::begin(vec), std::end(vec), ";");
  std::cout << str << std::endl;
}

