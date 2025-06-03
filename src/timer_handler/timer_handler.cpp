#include <mrs_lib/timer_handler.h>

namespace mrs_lib
{

// | ------------------------ ROSTimer ------------------------ |

/* ROSTimer:: constructors //{ */

ROSTimer::ROSTimer(const rclcpp::Node::SharedPtr& node, const rclcpp::Rate& rate, const std::function<void()>& callback) {

  this->node_ = node;

  this->timer_ = node_->create_timer(std::chrono::nanoseconds(rate.period()), callback);
}

ROSTimer::ROSTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, const std::function<void()>& callback) {

  this->node_ = opts.node;

  this->oneshot = opts.oneshot;

  this->callback_group_ = opts.callback_group;

  this->callback = callback;

  if (this->callback_group_) {
    this->timer_ = node_->create_timer(std::chrono::nanoseconds(rate.period()), callback, opts.callback_group.value());
  } else {
    this->timer_ = node_->create_timer(std::chrono::nanoseconds(rate.period()), callback);
  }

  if (!opts.autostart) {
    this->timer_->cancel();
  }
}


//}

/* stop() //{ */

void ROSTimer::stop() {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {

    timer_->cancel();

    while (rclcpp::ok() && !timer_->is_canceled()) {
      node_->get_clock()->sleep_for(std::chrono::duration<double>(0.001));
    }
  }
}

//}

/* start() //{ */

void ROSTimer::start() {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    timer_->reset();
  }
}

//}

/* setPeriod() //{ */

void ROSTimer::setPeriod(const rclcpp::Duration& duration) {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {

    timer_->cancel();

    if (this->callback_group_) {
      this->timer_ = node_->create_timer(std::chrono::duration<double>(duration.seconds()), callback, this->callback_group_.value());
    } else {
      this->timer_ = node_->create_timer(std::chrono::duration<double>(duration.seconds()), callback);
    }
  }
}

//}

/* setCallback() //{ */

void ROSTimer::setCallback([[maybe_unused]] const std::function<void()>& callback) {

  RCLCPP_ERROR(node_->get_logger(), "[mrs_lib::ROSTimer]: setCallback(...) is not implemented for ROSTimer! Check [mrs_lib/src/timer/timer.cpp].");
}
//}

/* running() //{ */

bool ROSTimer::running() {
  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    return !timer_->is_canceled();
  } else {
    return false;
  }
}

//}

// | ----------------------- ThreadTimer ---------------------- |

/* ThreadTimer constructors and destructors//{ */

ThreadTimer::ThreadTimer(const rclcpp::Node::SharedPtr& node, const rclcpp::Rate& rate, const std::function<void()>& callback) {

  this->impl_ = std::make_unique<Impl>(node, callback, rate, false);

  this->impl_->start();
}

ThreadTimer::ThreadTimer(const mrs_lib::TimerHandlerOptions& opts, const rclcpp::Rate& rate, const std::function<void()>& callback) {

  this->impl_ = std::make_unique<Impl>(opts.node, callback, rate, opts.oneshot);

  if (opts.autostart) {
    this->impl_->start();
  }
}

//}

/* TheadTimer::start() //{ */

void ThreadTimer::start() {

  if (impl_) {
    impl_->start();
  }
}

//}

/* ThreadTimer::stop() //{ */

void ThreadTimer::stop() {

  if (impl_) {
    impl_->stop();
  }
}

//}

/* ThreadTimer::setPeriod() //{ */

void ThreadTimer::setPeriod(const rclcpp::Duration& duration) {

  if (impl_) {
    impl_->setPeriod(duration);
  }
}

//}

/* ThreadTimer::setCallback() //{ */

void ThreadTimer::setCallback(const std::function<void()>& callback) {

  if (impl_) {
    impl_->setCallback(callback);
  }
}

//}

/* ThreadTimer::running() //{ */

bool ThreadTimer::running() {

  if (impl_) {
    return impl_->running_;
  } else {
    return false;
  }
}

//}

/* ThreadTimer::Impl //{ */

ThreadTimer::Impl::Impl(const rclcpp::Node::SharedPtr& node, const std::function<void()>& callback, const rclcpp::Rate& rate, const bool oneshot)
    : oneshot_(oneshot) {

  node_ = node;

  callback_ = callback;

  ending_  = false;
  running_ = false;

  delay_dur_ = rate.period();

  thread_ = std::thread(&ThreadTimer::Impl::threadFcn, this);
}

ThreadTimer::Impl::~Impl() {

  {
    // signal the thread to end
    std::scoped_lock lck(mutex_wakeup_, mutex_state_);
    ending_ = true;
    wakeup_cond_.notify_all();
  }

  // wait for it to die
  thread_.join();
}

void ThreadTimer::Impl::start() {

  std::scoped_lock lck(mutex_wakeup_, mutex_state_);

  if (!running_) {
    next_expected_ = node_->get_clock()->now() + rclcpp::Duration(delay_dur_);
    running_       = true;
    wakeup_cond_.notify_all();
  }
}

void ThreadTimer::Impl::stop() {

  std::scoped_lock lck(mutex_wakeup_, mutex_state_);
  running_ = false;
  wakeup_cond_.notify_all();
}

void ThreadTimer::Impl::setPeriod(const rclcpp::Duration& duration) {

  std::scoped_lock lock(mutex_wakeup_, mutex_state_);

  delay_dur_ = std::chrono::nanoseconds(duration.nanoseconds());

  // gracefully handle the special case
  if (duration.nanoseconds() == 0) {
    this->oneshot_ = true;
  }
}

void ThreadTimer::Impl::setCallback(const std::function<void()>& callback) {
  std::scoped_lock lock(mutex_state_);

  callback_ = callback;
}

//}

/* ThreadTimer::breakableSleep() method //{ */
bool ThreadTimer::Impl::breakableSleep(const rclcpp::Time& until) {

  while (rclcpp::ok() && node_->get_clock()->now() < until) {

    const std::chrono::nanoseconds                           dur{(until - node_->get_clock()->now()).nanoseconds()};
    const std::chrono::time_point<std::chrono::steady_clock> until_stl = std::chrono::steady_clock::now() + dur;
    std::unique_lock                                         lck(mutex_wakeup_);

    if (!ending_ && running_) {
      wakeup_cond_.wait_until(lck, until_stl);
    }

    // check the flags while mutex_state_ is locked
    if (ending_ || !running_) {
      return false;
    }
  }

  return true;
}
//}

/* ThreadTimer::Impl::threadFcn() //{ */

void ThreadTimer::Impl::threadFcn() {

  while (rclcpp::ok() && !ending_) {
    {
      std::unique_lock lck(mutex_wakeup_);

      // if the timer is not yet started, wait for condition variable notification
      if (!running_ && !ending_) {
        wakeup_cond_.wait(lck);
      }

      // The thread either got cv-notified, or running_ was already true, or it got woken up for some other reason.
      // Check if the reason is that we should end (and end if it is).
      if (ending_) {
        break;
      }

      // Check if the timer is still paused - probably was a spurious wake up (and restart the wait if it is).
      if (!running_) {
        continue;
      }
    }

    // If the flow got here, the thread should attempt to wait for the specified duration.
    // first, copy the delay we should wait for in a thread-safe manner
    rclcpp::Time next_expected;
    {
      std::scoped_lock lck(mutex_state_);
      next_expected = node_->get_clock()->now() + rclcpp::Duration(std::chrono::duration<double>(delay_dur_));
    }

    const bool interrupted = !breakableSleep(next_expected);

    if (interrupted) {
      continue;
    }

    {
      // make sure the state doesn't change here
      std::scoped_lock lck(mutex_state_);

      // Again, check if everything is OK (the state may have changed while the thread was a sleeping beauty).
      // Check if we should end (and end if it is so).
      if (ending_) {
        break;
      }

      // Check if the timer is paused (and skip if it is so).
      if (!running_) {
        continue;
      }

      if (oneshot_) {
        running_ = false;
      }

      // call the callback
      callback_();
    }
  }
}

//}

}  // namespace mrs_lib
