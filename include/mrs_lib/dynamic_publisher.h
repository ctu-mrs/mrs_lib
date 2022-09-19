#ifndef DYNAMIC_PUBLISHER_H
#define DYNAMIC_PUBLISHER_H

#include <ros/ros.h>
#include <mrs_lib/publisher_handler.h>

namespace mrs_lib
{

  class DynamicPublisher
  {
  public:
    DynamicPublisher() = default;
    DynamicPublisher(const ros::NodeHandle& nh);

    template <class T>
    void publish(const std::string name, const T& value);

  private:
    ros::NodeHandle m_nh;

    std::unordered_map<std::string, ros::Publisher> m_publishers;
  };

  DynamicPublisher::DynamicPublisher(const ros::NodeHandle& nh)
    : m_nh(nh)
  {
  }

  template <class T>
  void DynamicPublisher::publish(const std::string name, const T& value)
  {
    if (m_publishers.count(name) == 0)
      m_publishers.emplace(name, m_nh.advertise<T>(name, 10));

    m_publishers.at(name).publish(value);
  }

}  // namespace mrs_lib

#endif  // DYNAMIC_PUBLISHER_H
