// clang: MatousFormat

// Include the LKF header
#include <mrs_lib/mutex.h>
#include <iostream>

int main()
{
  std::mutex mtx;
  bool bvar = true;
  std::cout << "before: " << bvar << std::endl;
  auto [ret] = mrs_lib::get_set_mutexed(mtx,
      std::forward_as_tuple(bvar),
      std::forward_as_tuple(bvar),
      std::make_tuple(false)
      );
  std::cout << "after: " << bvar << ", read: " << ret << std::endl;
}



