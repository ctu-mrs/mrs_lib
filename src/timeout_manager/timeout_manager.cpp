#include <mrs_lib/timeout_manager.h>

namespace mrs_lib
{

  TimeoutManager::TimeoutManager(const ros::NodeHandle& nh, const ros::Rate& update_rate)
    : m_last_id(0)
  {
    m_main_timer = nh.createTimer(update_rate, &TimeoutManager::main_timer_callback, this);
  }


  TimeoutManager::timeout_id_t TimeoutManager::registerNew(const ros::Duration& timeout, const callback_t& callback, const ros::Time& last_reset, const bool oneshot, const bool autostart)
  {
    std::scoped_lock lck(m_mtx);
    const auto new_id = m_timeouts.size();
    const timeout_info_t new_info = {oneshot, autostart, callback, timeout, last_reset, last_reset};
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
    m_timeouts.at(id).started = false;
  }

  void TimeoutManager::start(const timeout_id_t id, const ros::Time& time)
  {
    std::scoped_lock lck(m_mtx);
    m_timeouts.at(id).started = true;
    m_timeouts.at(id).last_reset = time;
  }

  void TimeoutManager::pauseAll()
  {
    std::scoped_lock lck(m_mtx);
    for (auto& timeout_info : m_timeouts)
      timeout_info.started = false;
  }

  void TimeoutManager::startAll(const ros::Time& time)
  {
    std::scoped_lock lck(m_mtx);
    for (auto& timeout_info : m_timeouts)
    {
      timeout_info.started = true;
      timeout_info.last_reset = time;
    }
  }

  void TimeoutManager::change(const timeout_id_t id, const ros::Duration& timeout, const callback_t& callback, const ros::Time& last_reset, const bool oneshot, const bool autostart)
  {
    std::scoped_lock lck(m_mtx);
    auto& timeout_info = m_timeouts.at(id);
    timeout_info.oneshot = oneshot;
    timeout_info.started = autostart;
    timeout_info.timeout = timeout;
    timeout_info.callback = callback;
    timeout_info.last_reset = last_reset;
  }

  ros::Time TimeoutManager::lastReset(const timeout_id_t id)
  {
    std::scoped_lock lck(m_mtx);
    return m_timeouts.at(id).last_reset;
  }

  bool TimeoutManager::started(const timeout_id_t id)
  {
    std::scoped_lock lck(m_mtx);
    return m_timeouts.at(id).started;
  }

  void TimeoutManager::main_timer_callback([[maybe_unused]] const ros::TimerEvent &evt)
  {
    const auto now = ros::Time::now();

    std::scoped_lock lck(m_mtx);
    for (auto& timeout_info : m_timeouts)
    {
      // don't worry, this'll get optimized out by the compiler
      const bool started = timeout_info.started;
      const bool last_reset_timeout = now - timeout_info.last_reset >= timeout_info.timeout;
      const bool last_callback_timeout = now - timeout_info.last_callback >= timeout_info.timeout;
      if (started && last_reset_timeout && last_callback_timeout)
      {
        timeout_info.callback(timeout_info.last_reset);
        timeout_info.last_callback = now;
        // if the timeout is oneshot, pause it
        if (timeout_info.oneshot)
          timeout_info.started = false;
      }
    }
  }
}
