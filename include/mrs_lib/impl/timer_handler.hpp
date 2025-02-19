#ifndef MRS_TIMER_HPP
#define MRS_TIMER_HPP

#ifndef MRS_TIMER_H
#include <mrs_lib/timer_handler.h>
#endif

#include <mutex>

namespace mrs_lib
{

// | ----------------------- ThreadTimer ---------------------- |

/* class ThreadTimer::Impl //{ */

class mrs_lib::ThreadTimer::Impl {
public:
  Impl(const rclcpp::Node::SharedPtr& node, const std::function<void()>& callback, const rclcpp::Rate& rate, const bool oneshot);

  ~Impl();

  void start();
  void stop();
  void setPeriod(const rclcpp::Duration& duration);
  void setCallback(const std::function<void()>& callback);

  friend class ThreadTimer;

  // to keep rule of five since we have a custom destructor
  Impl(const Impl&)            = delete;
  Impl(Impl&&)                 = delete;
  Impl& operator=(const Impl&) = delete;
  Impl& operator=(Impl&&)      = delete;

private:
  std::thread           thread_;
  std::function<void()> callback_;

  rclcpp::Node::SharedPtr node_;

  bool oneshot_;

  bool breakableSleep(const rclcpp::Time& until);
  void threadFcn();

  std::mutex               mutex_wakeup_;
  std::condition_variable  wakeup_cond_;
  std::recursive_mutex     mutex_state_;
  bool                     running_;
  std::chrono::nanoseconds delay_dur_;
  bool                     ending_;
  rclcpp::Time             next_expected_;
};

//}

}  // namespace mrs_lib

#endif  // MRS_TIMER_HPP
