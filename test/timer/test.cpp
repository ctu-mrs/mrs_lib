#include <memory>
#include <mrs_lib/timer.h>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace mrs_lib;
using namespace std;

std::unique_ptr<mrs_lib::MRSTimer> timer;

struct obj_t
{
  ros::Rate r = ros::Rate(100.0);
  int n_cbks = 0;
  bool cbks_in_time = true;
  bool cbks_ok = true;
  void callback(const ros::TimerEvent& evt)
  {
    n_cbks++;
    if (std::abs((evt.current_expected - evt.current_real).toSec()) > r.expectedCycleTime().toSec()/2.0)
    {
      ROS_ERROR_STREAM("Callback did not come in time! Expected: " << evt.current_expected << ", received: " << evt.current_real);
      cbks_in_time = false;
    }

    if (!timer->running())
      cbks_ok = false;
  }
};

/* TEST(TESTSuite, thread_timer_test) //{ */

TEST(TESTSuite, thread_timer_test) {

  ros::NodeHandle nh("~");

  obj_t cbk_obj;
  timer = std::make_unique<mrs_lib::ThreadTimer>(nh, cbk_obj.r, &obj_t::callback, &cbk_obj, false, true);

  const ros::Time start = ros::Time::now();
  const ros::Duration test_dur(1.0);
  while (ros::Time::now() - start < test_dur)
  {
    ros::Rate(1000.0).sleep();
    ros::spinOnce();
  }
  timer->stop();

  const double expected_cbks = test_dur.toSec() / cbk_obj.r.expectedCycleTime().toSec();
  const bool callbacks_in_time = cbk_obj.cbks_in_time;
  const bool callback_while_not_running = !cbk_obj.cbks_ok;

  EXPECT_TRUE(callbacks_in_time);
  EXPECT_FALSE(callback_while_not_running);
  EXPECT_LE(std::abs(cbk_obj.n_cbks - expected_cbks), 2);

  {
    // test correct destruction
    cbk_obj.cbks_ok = true;
    const ros::Time start = ros::Time::now();
    timer->setPeriod(ros::Duration(50.0));
    timer->start();
    timer = nullptr;
    const ros::Time destroy = ros::Time::now();
    const bool callback_while_destroyed = !cbk_obj.cbks_ok;
    EXPECT_FALSE(callback_while_destroyed);
    EXPECT_LE((destroy - start).toSec(), 1.0);
  }
}

//}

/* TEST(TESTSuite, ros_timer_test) //{ */

TEST(TESTSuite, ros_timer_test) {

  ros::NodeHandle nh("~");

  obj_t cbk_obj;
  timer = std::make_unique<mrs_lib::ROSTimer>(nh, cbk_obj.r, &obj_t::callback, &cbk_obj, false, true);

  const ros::Time start = ros::Time::now();
  const ros::Duration test_dur(1.0);
  while (ros::Time::now() - start < test_dur)
  {
    ros::Rate(1000.0).sleep();
    ros::spinOnce();
  }
  timer->stop();

  const double expected_cbks = test_dur.toSec() / cbk_obj.r.expectedCycleTime().toSec();
  const bool callbacks_in_time = cbk_obj.cbks_in_time;
  const bool callback_while_not_running = !cbk_obj.cbks_ok;

  EXPECT_TRUE(callbacks_in_time);
  EXPECT_FALSE(callback_while_not_running);
  EXPECT_LE(std::abs(cbk_obj.n_cbks - expected_cbks), 2);

  {
    // test correct destruction
    cbk_obj.cbks_ok = true;
    const ros::Time start = ros::Time::now();
    timer->setPeriod(ros::Duration(50.0));
    timer->start();
    timer = nullptr;
    const ros::Time destroy = ros::Time::now();
    const bool callback_while_destroyed = !cbk_obj.cbks_ok;
    EXPECT_FALSE(callback_while_destroyed);
    EXPECT_LE((destroy - start).toSec(), 1.0);
  }
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "TimerTest");
  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

