#include <mrs_lib/dynamic_publisher.h>

namespace mrs_lib
{
  DynamicPublisher::DynamicPublisher()
  {
    m_impl = std::make_unique<impl>();
  }

  DynamicPublisher::DynamicPublisher(const ros::NodeHandle& nh)
  {
    m_impl = std::make_unique<impl>(nh);
  }
}
