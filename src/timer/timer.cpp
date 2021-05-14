#include <mrs_lib/timer.h>

namespace mrs_lib
{

// | ------------------------ ROSTimer ------------------------ |

/* stop() //{ */

void ROSTimer::stop() {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    timer_->stop();
  }
}

//}

/* start() //{ */

void ROSTimer::start() {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    timer_->start();
  }
}

//}

/* setPeriod() //{ */

void ROSTimer::setPeriod(const ros::Duration& duration, const bool reset) {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    timer_->setPeriod(duration, reset);
  }
}

//}

// | ----------------------- ThreadTimer ---------------------- |

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

void ThreadTimer::setPeriod(const ros::Duration& duration, [[maybe_unused]] const bool reset) {

  if (impl_) {
    impl_->setPeriod(duration, reset);
  }
}

//}

/* ThreadTimer::Impl //{ */

ThreadTimer::Impl::Impl(const std::function<void(const ros::TimerEvent&)>& callback, const ros::Duration& delay_dur, const bool oneshot)
  : callback_(callback), oneshot_(oneshot), delay_dur_(delay_dur)
{
  ending_  = false;
  running_ = false;

  last_real_     = ros::Time(0);
  last_expected_ = ros::Time(0);
  next_expected_ = ros::Time(0);

  thread_ = std::thread(&ThreadTimer::Impl::threadFcn, this);
}

ThreadTimer::Impl::~Impl()
{
  {
    // signal the thread to end
    std::scoped_lock lck(mutex_state_);
    ending_ = true;
    start_cond_.notify_all();
  }
  // wait for it to die
  thread_.join();
}

void ThreadTimer::Impl::start()
{
  std::scoped_lock lck(mutex_state_);
  if (!running_)
  {
    next_expected_ = ros::Time::now() + delay_dur_;
    running_ = true;
    start_cond_.notify_all();
  }
}

void ThreadTimer::Impl::stop()
{
  std::scoped_lock lck(mutex_state_);
  running_ = false;
}

void ThreadTimer::Impl::setPeriod(const ros::Duration& duration, [[maybe_unused]] const bool reset)
{
  std::scoped_lock lock(mutex_state_);
  delay_dur_ = duration;
  // gracefully handle the special case
  if (duration == ros::Duration(0))
    this->oneshot_  = true;
}

//}

/* ThreadTimer::Impl::threadFcn() //{ */

void ThreadTimer::Impl::threadFcn()
{
  while (ros::ok() && !ending_)
  {
    {
      std::unique_lock lck(mutex_state_);
      // if the timer is not yet started, wait for condition variable notification
      if (!running_)
        start_cond_.wait(lck);
      // The thread either got cv-notified, or running_ was already true, or it got woken up for some other reason.
      // Check if the reason is that we should end (and end if it is).
      if (ending_)
        break;
      // Check if the timer is still paused - probably was a spurious wake up (and restart the wait if it is).
      if (!running_)
        continue;
    }

    // If the flow got here, the thread should attempt to wait for the specified duration.
    // first, copy the delay we should wait for in a thread-safe manner
    ros::Duration delay;
    {
      std::scoped_lock lck(mutex_state_);
      delay = next_expected_ - ros::Time::now();
    }
    delay.sleep();

    // prepare the timer event (outside the locked scope so that it is usable by the callback without holding the mutex)
    ros::TimerEvent timer_event;
    {
      // make sure the state doesn't change here
      std::scoped_lock lck(mutex_state_);
      // Again, check if everything is OK (the state may have changed while the thread was a sleeping beauty).
      // Check if we should end (and end if it is so).
      if (ending_)
        break;
      // Check if the timer is paused (and skip if it is so).
      if (!running_)
        continue;

      // If all is fine and dandy, actually run the callback function!
      // if the timer is a oneshot-type, automatically pause it
      // and do not fill out the expected fields in timer_event (we had no expectations...)
      const ros::Time now = ros::Time::now();
      if (oneshot_)
      {
        running_ = false;
        timer_event.last_real        = last_real_;
        timer_event.current_real     = now;
      }
      else
      {
        timer_event.last_expected    = last_expected_;
        timer_event.last_real        = last_real_;
        timer_event.current_expected = next_expected_;
        timer_event.current_real     = now;

        last_expected_ = next_expected_;
        next_expected_ = now + delay_dur_;
      }
      last_real_ = now;
    }
    callback_(timer_event);
  }
}

//}

}  // namespace mrs_lib
