#include <ros/duration.h>
#include <memory>
#include <mrs_lib/timer.h>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>
#include <mutex>
#include <mrs_lib/utils.h>

using namespace mrs_lib;
using namespace std;

std::unique_ptr<mrs_lib::MRSTimer> timer = nullptr;
std::atomic<bool> test_stop_from_cbk;

struct obj_t
{
  ros::Rate r = ros::Rate(50.0);
  int n_cbks = 0;
  bool cbks_in_time = true;
  bool null_cbk = false;
  bool cbks_ok = true;
  std::mutex cbk_running_mtx;
  std::atomic<bool> cbk_running = false;

  void callback(const ros::TimerEvent& evt)
  {
    std::scoped_lock lck(cbk_running_mtx);
    mrs_lib::AtomicScopeFlag running(cbk_running);
    n_cbks++;
    const ros::Duration dt = evt.current_real - evt.current_expected;
    // this is quite a large tolerance, but the ROS timer actually isn't capable of performing better, so it has to be
    if (std::abs(dt.toSec()) > r.expectedCycleTime().toSec()/2.0)
    {
      ROS_ERROR_STREAM("Callback did not come in time! Expected: " << evt.current_expected << ", received: " << evt.current_real << " (period: " << r.expectedCycleTime() << ", difference: " << dt << ")");
      cbks_in_time = false;
    }

    if (!timer)
    {
      ROS_ERROR_STREAM("Callback called while timer is already destroyed!");
      null_cbk = true;
      return;
    }

    if (!timer->running())
    {
      ROS_ERROR_STREAM("Callback called while timer is not running!");
      cbks_ok = false;
    }

    if (test_stop_from_cbk)
    {
      timer->stop();
      cout << "\tStopping timer from callback.";
    }
  }
};

void do_test(const bool use_threadtimer)
{
  ros::NodeHandle nh("~");

  test_stop_from_cbk = false;
  obj_t cbk_obj;
  cout << "\tConstructing timer\n";
  if (use_threadtimer)
    timer = std::make_unique<mrs_lib::ThreadTimer>(nh, cbk_obj.r, &obj_t::callback, &cbk_obj, false, false);
  else
    timer = std::make_unique<mrs_lib::ROSTimer>(nh, cbk_obj.r, &obj_t::callback, &cbk_obj, false, false);

  {
    cout << "\tTesting callback period\n";
    timer->start();
    const ros::Time start = ros::Time::now();
    const ros::Duration test_dur(0.5);
    while (ros::Time::now() - start < test_dur)
    {
      ros::Rate(1000.0).sleep();
      ros::spinOnce();
    }
    timer->stop();
    EXPECT_FALSE(cbk_obj.cbk_running);

    const double expected_cbks = test_dur.toSec() / cbk_obj.r.expectedCycleTime().toSec();
    const bool callbacks_in_time = cbk_obj.cbks_in_time;
    const bool callback_while_not_running = !cbk_obj.cbks_ok;

    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE(std::abs(cbk_obj.n_cbks - expected_cbks), 2);
    EXPECT_TRUE(callbacks_in_time);

    cbk_obj.n_cbks = 0;
    ros::Duration(cbk_obj.r.cycleTime().toSec()*2).sleep();
    const bool no_callbacks_after_stopped = cbk_obj.n_cbks == 0;
    EXPECT_TRUE(no_callbacks_after_stopped);
  }

  {
    cout << "\tTesting stop from callback\n";
    cbk_obj.n_cbks = 0;
    test_stop_from_cbk = true;
    timer->start();
    // wait for one callback to be called
    while (!cbk_obj.n_cbks)
    {
      ros::Rate(1000.0).sleep();
      ros::spinOnce();
    }
    // wait for the callback to end
    {
      std::scoped_lock lck(cbk_obj.cbk_running_mtx);
      cbk_obj.n_cbks = 0;
      const bool timer_stopped_from_callback = !timer->running();
      EXPECT_TRUE(timer_stopped_from_callback);
    }

    ros::Duration(cbk_obj.r.cycleTime().toSec()*2).sleep();
    const bool no_callbacks_after_stopped = cbk_obj.n_cbks == 0;
    EXPECT_TRUE(no_callbacks_after_stopped);
  }

  {
    cout << "\tTesting destructor\n";
    cbk_obj.cbks_ok = true;
    const ros::Time start = ros::Time::now();
    const ros::Duration period = ros::Duration(50.0);
    cbk_obj.r = ros::Rate(period);
    timer->setPeriod(period);
    timer->start();
    timer = nullptr;
    const ros::Time destroyed = ros::Time::now();
    const bool callback_while_destroyed = cbk_obj.null_cbk;
    const bool callback_while_not_running = !cbk_obj.cbks_ok;

    EXPECT_FALSE(callback_while_destroyed);
    EXPECT_FALSE(callback_while_not_running);
    EXPECT_LE((destroyed - start).toSec(), 1.0);
  }
}

/* TEST(TESTSuite, ros_timer_test) //{ */

TEST(TESTSuite, ros_timer_test)
{
  cout << "Testing ROSTimer\n";
  for (int it = 0; it < 8; it++)
    do_test(false);
}

//}

/* TEST(TESTSuite, thread_timer_test) //{ */

TEST(TESTSuite, thread_timer_test)
{
  cout << "Testing ThreadTimer\n";
  for (int it = 0; it < 8; it++)
    do_test(true);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "TimerTest");
  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

