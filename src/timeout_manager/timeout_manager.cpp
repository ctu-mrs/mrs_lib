#include <mrs_lib/timeout_manager.h>

namespace mrs_lib
{

  TimeoutManager::TimeoutManager(const ros::NodeHandle& nh, const ros::Rate& update_rate)
    : m_last_id(0)
  {
    nh.createTimer(update_rate, &TimeoutManager::main_timer_callback, this);
  }


  TimeoutManager::timeout_id_t TimeoutManager::registerNew(const ros::Duration& timeout, const callback_t& callback, const ros::Time& last_reset)
  {
    std::scoped_lock lck(m_mtx);
    const auto new_id = m_timeouts.size();
    const timeout_info_t new_info = {false, callback, timeout, last_reset, last_reset};
    m_timeouts.push_back(new_info);
    return new_id;
  }

  void TimeoutManager::reset(const timeout_id_t id, const ros::Time& time)
  {
    std::scoped_lock lck(m_mtx);
    m_timeouts.at(id).last_reset = time;
  }

  void TimeoutManager::pause(const timeout_id_t id)
  {
    std::scoped_lock lck(m_mtx);
    m_timeouts.at(id).paused = true;
  }

  void TimeoutManager::start(const timeout_id_t id, const ros::Time& time)
  {
    std::scoped_lock lck(m_mtx);
    m_timeouts.at(id).paused = false;
    m_timeouts.at(id).last_reset = time;
  }

  void TimeoutManager::change(const timeout_id_t id, const ros::Duration& timeout, const callback_t& callback, const ros::Time& last_reset)
  {
    std::scoped_lock lck(m_mtx);
    auto& timeout_info = m_timeouts.at(id);
    timeout_info.timeout = timeout;
    timeout_info.callback = callback;
    timeout_info.last_reset = last_reset;
  }

  ros::Time TimeoutManager::lastReset(const timeout_id_t id)
  {
    std::scoped_lock lck(m_mtx);
    return m_timeouts.at(id).last_reset;
  }

  void TimeoutManager::main_timer_callback([[maybe_unused]] const ros::TimerEvent &evt)
  {
    const auto now = ros::Time::now();

    std::scoped_lock lck(m_mtx);
    for (auto& timeout_info : m_timeouts)
    {
      // don't worry, this'll get optimized
      const bool paused = timeout_info.paused;
      const bool last_reset_timeout = now - timeout_info.last_reset > timeout_info.timeout;
      const bool last_callback_timeout = now - timeout_info.last_callback > timeout_info.timeout;
      if (paused && last_reset_timeout && last_callback_timeout)
      {
        timeout_info.callback(timeout_info.last_reset);
        timeout_info.last_callback = now;
      }
    }
  }
}
