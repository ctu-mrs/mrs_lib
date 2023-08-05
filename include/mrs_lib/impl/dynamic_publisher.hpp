class DynamicPublisher::impl
{
public:
  impl() = default;

  impl(const ros::NodeHandle& nh) : m_nh(nh)
  {
  }

  template <class T>
  void publish(const std::string name, const T& msg)
  {
    std::scoped_lock lck(m_mtx);
    const std::string msg_md5 = ros::message_traits::MD5Sum<T>::value();
    const std::string msg_datatype = ros::message_traits::DataType<T>::value();
    if (m_publishers.count(name) == 0)
      m_publishers.emplace(name, pub_info_t{m_nh.advertise<T>(name, 10), msg_md5, msg_datatype});

    const auto& pub_info = m_publishers.at(name);
    if (pub_info.msg_md5 == "*" ||
        msg_md5 == "*" ||
        pub_info.msg_md5 == msg_md5)
    {
      pub_info.pub.publish(msg);
    }
    else
    {
      ROS_ERROR_STREAM("[DynamicPublisher]: Trying to publish message of type [" << msg_datatype << "/" << msg_md5
                    << "] on a publisher with type [" << pub_info.datatype << "/" << pub_info.msg_md5 << "], ignoring!");
    }
  }

private:
  struct pub_info_t
  {
    ros::Publisher pub;
    std::string msg_md5;
    std::string datatype;
  };
  std::mutex m_mtx;
  ros::NodeHandle m_nh;
  std::unordered_map<std::string, pub_info_t> m_publishers;
};

template <class T>
void DynamicPublisher::publish(const std::string name, const T& msg)
{
  m_impl->publish(name, msg);
}
