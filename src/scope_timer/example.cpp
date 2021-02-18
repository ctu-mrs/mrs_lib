#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/scope_timer.h>

int main(int argc, char * argv[])
{
  {
    mrs_lib::ScopeTimer tim("test");
    for (int it = 0; it < 1e6; it++);
    tim.checkpoint("first for");
    for (int it = 0; it < 1e5; it++);
  }

  {
    rclcpp::init(argc, argv);
    std::shared_ptr nh = std::make_shared<rclcpp::Node>("scope_timer_example");
    mrs_lib::ScopeTimer tim(*nh, "test");
    for (int it = 0; it < 1e6; it++);
    tim.checkpoint("first for");
    for (int it = 0; it < 1e5; it++);
  }

  return 0;
}
