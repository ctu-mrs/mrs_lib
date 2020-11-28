#include <mrs_lib/timer.h>

namespace mrs_lib
{

// | ------------------------ ROSTimer ------------------------ |

/* ROSTimer constructors //{ */

ROSTimer::ROSTimer(void) {
}

ROSTimer::~ROSTimer(void) {
}

ROSTimer::ROSTimer(const ROSTimer& other) {
  if (other.timer_) {
    this->timer_ = other.timer_;
  }
}

//}

/* ROSTimer::operator= //{ */

ROSTimer& ROSTimer::operator=(const ROSTimer& other) {

  if (this == &other) {
    return *this;
  }

  if (other.timer_) {
    this->timer_ = other.timer_;
  }

  return *this;
}

/* //} */

/* stop() //{ */

void ROSTimer::stop(void) {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    timer_->stop();
  }
}

//}

/* start() //{ */

void ROSTimer::start(void) {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    timer_->start();
  }
}

//}

/* setPeriod() //{ */

void ROSTimer::setPeriod(const ros::Duration& duration, const bool& reset) {

  std::scoped_lock lock(mutex_timer_);

  if (timer_) {
    timer_->setPeriod(duration, reset);
  }
}

//}

// | ----------------------- ThreadTimer ---------------------- |

/* ThreadTimer constructors and destructors//{ */

ThreadTimer::ThreadTimer() {
}

ThreadTimer::~ThreadTimer() {
}

ThreadTimer::ThreadTimer(const ThreadTimer& other) {

  impl_ = other.impl_;
}

//}

/* ThreadTimer::operator= //{ */

ThreadTimer& ThreadTimer::operator=(const ThreadTimer& other) {

  if (this == &other) {
    return *this;
  }

  this->impl_ = other.impl_;

  return *this;
}

//}

/* TheadTimer::start() //{ */

void ThreadTimer::start(void) {

  if (impl_) {
    impl_->start();
  }
}

//}

/* ThreadTimer::stop() //{ */

void ThreadTimer::stop(void) {

  if (impl_) {
    impl_->stop();
  }
}

//}

/* ThreadTimer::setPeriod() //{ */

void ThreadTimer::setPeriod(const ros::Duration& duration, [[maybe_unused]] const bool& reset) {

  if (impl_) {
    impl_->setPeriod(duration, reset);
  }
}

//}

/* ThreadTimer::Impl //{ */

ThreadTimer::Impl::~Impl() {
}

ThreadTimer::Impl::Impl() : rate_(ros::Rate(1.0)), duration_(ros::Duration(0.0)) {

  this->running_ = false;
  this->oneshot_ = false;
  this->shoot_   = false;

  this->last_real_     = ros::Time(0);
  this->last_expected_ = ros::Time(0);
  this->next_expected_ = ros::Time(0);
}

void ThreadTimer::Impl::start(void) {

  this->stop();

  if (!running_) {

    running_ = true;

    thread_ = std::thread(&Impl::threadFcn, this);

    thread_.detach();
  }

  if (oneshot_) {
    {
      std::lock_guard<std::mutex> lock(mutex_oneshot_);
      shoot_ = true;
    }
    oneshot_cond_.notify_one();
  }
}

void ThreadTimer::Impl::stop(void) {

  if (!oneshot_) {
    running_ = false;
  }
}

void ThreadTimer::Impl::setPeriod(const ros::Duration& duration, [[maybe_unused]] const bool& reset) {

  std::scoped_lock lock(mutex_time_);

  if (duration.toSec() > 0) {
    this->rate_ = 1.0 / duration.toSec();
  } else {
    this->oneshot_  = true;
    this->rate_     = ros::Rate(1.0);
    this->duration_ = duration;
  }

  rate_changed_ = true;
}

//}

/* ThreadTimer::Impl::threadFcn() //{ */

void ThreadTimer::Impl::threadFcn(void) {

  if (oneshot_) {

    ros::Duration temp_duration;

    while (ros::ok() && running_) {

      std::unique_lock<std::mutex> lock(mutex_oneshot_);
      oneshot_cond_.wait(lock);

      {
        std::scoped_lock lock(mutex_time_);

        temp_duration = duration_;
      }

      temp_duration.sleep();

      ros::TimerEvent timer_event;
      callback_(timer_event);

      lock.unlock();
    }

  } else {

    ros::Rate temp_rate = rate_;

    while (ros::ok() && running_) {

      ros::TimerEvent timer_event;
      timer_event.last_expected    = last_expected_;
      timer_event.last_real        = last_real_;
      timer_event.current_expected = next_expected_;
      timer_event.current_real     = ros::Time::now();

      callback_(timer_event);

      last_expected_ = next_expected_;
      last_real_     = ros::Time::now();

      next_expected_ = ros::Time::now() + rate_.expectedCycleTime();

      temp_rate.sleep();

      if (rate_changed_) {
        std::scoped_lock lock(mutex_time_);
        temp_rate = rate_;
      }
    }
  }
}

//}

}  // namespace mrs_lib
