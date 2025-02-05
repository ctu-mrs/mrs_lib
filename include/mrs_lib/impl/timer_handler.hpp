#ifndef MRS_TIMER_HPP
#define MRS_TIMER_HPP

#include <mrs_lib/timer_handler.h>

namespace mrs_lib
{

// | ------------------------ ROSTimer ------------------------ |

/* ROSTimer constructors //{ */

// duration + oneshot + autostart
#include <chrono>
#include <mutex>

  ROSTimer::ROSTimer(const rclcpp::Node::SharedPtr& node, const double& rate, const std::function<void()>& callback)
  {

    this->node_ = node;

    this->timer_ = node_->create_timer(std::chrono::duration<double>(1.0 / rate), callback);
  }

  ROSTimer::ROSTimer(const mrs_lib::TimerHandlerOptions& opts, const double& rate, const std::function<void()>& callback)
  {

    this->node_ = opts.node;

    this->oneshot = opts.oneshot;

    this->callback_group_ = opts.callback_group;

    this->callback = callback;

    if (this->callback_group_)
    {
      this->timer_ = node_->create_timer(std::chrono::duration<double>(1.0 / rate), callback, opts.callback_group.value());
    } else
    {
      this->timer_ = node_->create_timer(std::chrono::duration<double>(1.0 / rate), callback);
    }

    if (!opts.autostart)
    {
      this->timer_->cancel();
    }
  }

  //}

  // | ----------------------- ThreadTimer ---------------------- |

  /* class ThreadTimer::Impl //{ */

  class mrs_lib::ThreadTimer::Impl
  {
  public:
    Impl(const rclcpp::Node::SharedPtr& node, const std::function<void()>& callback, const double& rate, const bool oneshot);

    ~Impl();

    void start();
    void stop();
    void setPeriod(const double& duration);
    void setCallback(const std::function<void()>& callback);

    friend class ThreadTimer;

    // to keep rule of five since we have a custom destructor
    Impl(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl& operator=(Impl&&) = delete;

  private:
    std::thread thread_;
    std::function<void()> callback_;

    rclcpp::Node::SharedPtr node_;

    bool oneshot_;

    bool breakableSleep(const rclcpp::Time& until);
    void threadFcn();

    std::mutex mutex_wakeup_;
    std::condition_variable wakeup_cond_;
    std::recursive_mutex mutex_state_;
    bool running_;
    double delay_dur_;
    bool ending_;
    rclcpp::Time next_expected_;
  };

  //}

  /* ThreadTimer constructors and destructors//{ */

  ThreadTimer::ThreadTimer(const rclcpp::Node::SharedPtr& node, const double& rate, const std::function<void()>& callback)
  {

    this->impl_ = std::make_unique<Impl>(node, callback, rate, false);

    this->impl_->start();
  }

  ThreadTimer::ThreadTimer(const mrs_lib::TimerHandlerOptions& opts, const double& rate, const std::function<void()>& callback)
  {

    this->impl_ = std::make_unique<Impl>(opts.node, callback, rate, opts.oneshot);

    if (opts.autostart)
    {
      this->impl_->start();
    }
  }

  //}
  //
}  // namespace mrs_lib

#endif  // MRS_TIMER_HPP
