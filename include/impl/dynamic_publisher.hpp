class DynamicPublisher::impl
{
public:
  impl() = default;

  impl(const ros::NodeHandle& nh) : m_nh(nh)
  {
  }

  template <class T>
  void publish(const std::string name, const T& value)
  {
    std::scoped_lock lck(m_mtx);
    if (m_publishers.count(name) == 0)
      m_publishers.emplace(name, m_nh.advertise<T>(name, 10));

    m_publishers.at(name).publish(value);
  }

private:
  std::mutex m_mtx;
  ros::NodeHandle m_nh;
  std::unordered_map<std::string, ros::Publisher> m_publishers;
};

template <class T>
void DynamicPublisher::publish(const std::string name, const T& value)
{
  m_impl->publish(name, value);
}
