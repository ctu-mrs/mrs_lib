#include <ros/duration.h>
#include <memory>
#include <mrs_lib/timeout_manager.h>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>
#include <mutex>
#include <mrs_lib/utils.h>

using namespace mrs_lib;
using namespace std;

std::unique_ptr<ros::NodeHandle> nh;
std::unique_ptr<mrs_lib::TimeoutManager> tom = nullptr;
std::atomic<bool> test_stop_from_cbk = false;

struct obj_t
{
  // error flags
  std::mutex mtx;
  int n_cbks = 0;
  bool null_cbk = false;
  bool cbk_not_running = false;
  std::atomic_bool cbk_running = false;
  ros::Duration max_dt_err = ros::Duration(0.0);
  ros::Duration avg_dt_err = ros::Duration(0.0);

  // parameters
  bool check_dt_err = false;
  ros::Duration desired_dt;
  ros::Duration max_expected_dt_err;
  mrs_lib::TimeoutManager::timeout_id_t timeout_id;

  obj_t(const ros::Duration& desired_dt, const ros::Duration& max_expected_dt_err)
    : desired_dt(desired_dt), max_expected_dt_err(max_expected_dt_err)
  {
  }

  void set_timeout_id(mrs_lib::TimeoutManager::timeout_id_t new_timeout_id)
  {
    std::scoped_lock lck(mtx);
    timeout_id = new_timeout_id;
  }

  void callback(const ros::Time& last_update)
  {
    mrs_lib::AtomicScopeFlag flg(cbk_running);
    std::scoped_lock lck(mtx);
    const auto now = ros::Time::now();
    n_cbks++;

    if (!tom)
    {
      ROS_ERROR_STREAM("Callback called while timer is already destroyed!");
      null_cbk = true;
      return;
    }

    if (check_dt_err)
    {
      const ros::Duration dt = now - last_update;
      const auto dt_err = dt - desired_dt;
      if (dt_err < ros::Duration(0))
        ROS_ERROR_STREAM("Callback has a larger/smaller delay than expected: " << dt << "s (should be " << desired_dt << "s). Error: " << dt_err << "s (max. expected: " << max_expected_dt_err << "s)!");
      if (dt_err > max_expected_dt_err)
        ROS_ERROR_STREAM("Callback has a larger/smaller delay than expected: " << dt << "s (should be " << desired_dt << "s). Error: " << dt_err << "s (max. expected: " << max_expected_dt_err << "s)!");
      if (dt_err > max_dt_err || n_cbks == 1)
        max_dt_err = dt_err;
      avg_dt_err = (dt_err + avg_dt_err*(n_cbks-1)) * (1.0/n_cbks);
      tom->reset(timeout_id, now);
    }

    if (!tom->started(timeout_id))
    {
      ROS_ERROR_STREAM("Callback called while timer is not running!");
      cbk_not_running = true;
    }

    if (test_stop_from_cbk)
    {
      tom->pause(timeout_id);
      /* cout << "\tStopping timer from callback."; */
    }
  }
};

TEST(TESTSuite, timeout_test)
{
  const ros::Duration update_period(0.001); // values lower than 1ms reach ROS's capabilities
  const ros::Duration spin_period(0.0005);
  const ros::Duration max_expected_delay = update_period + spin_period;
  std::array<ros::Duration, 4> tos = { ros::Duration{ 0.1 }, ros::Duration{ 0.01 }, ros::Duration{ 0.001 }, ros::Duration{ 0.001 } };

  tom = std::make_unique<mrs_lib::TimeoutManager>(*nh, ros::Rate(update_period));
  std::array<obj_t, 4> objs = {obj_t(tos[0], max_expected_delay), obj_t(tos[1], max_expected_delay), obj_t(tos[2], max_expected_delay), obj_t(tos[3], max_expected_delay)};

  const auto now = ros::Time::now();
  for (int it = 0; it < 4; it++)
  {
    mrs_lib::TimeoutManager::callback_t cbk = std::bind(&obj_t::callback, &objs[it], std::placeholders::_1);
    objs[it].set_timeout_id(tom->registerNew(tos[it], cbk, now, false, false));
  }

  {
    cout << "\tTesting callback period\n";
    const ros::Time start = ros::Time::now();
    tom->startAll(start);
    const ros::Duration test_dur(0.5);
    while (ros::Time::now() - start < test_dur)
    {
      spin_period.sleep();
      ros::spinOnce();
    }
    tom->pauseAll();

    for (int it = 0; it < 4; it++)
    {
      auto& obj = objs[it];
      auto& to = tos[it];
      cout << "\tChecking object " << it << std::endl;
      EXPECT_FALSE(obj.cbk_running);
      std::scoped_lock lck(obj.mtx);

      const int expected_cbks = std::floor(test_dur.toSec() / to.toSec());
      const bool callback_while_not_running = obj.cbk_not_running;
      const bool callback_while_tm_null = obj.null_cbk;
      // there will be some "missed" callbacks due to cumulative aliasing of when TimeoutManager's timer runs
      // and when the callbacks should actually happen, but this should not be more than a certain amount
      // given by the TimeoutManager's update period (the smaller the better) and the callback period (the higher the better)
      const double max_miss_s = expected_cbks * update_period.toSec();
      const int max_misses = std::ceil(max_miss_s / to.toSec());
      cout << "Max. misses: " << max_misses << ", actual misses: " << (expected_cbks - obj.n_cbks) << std::endl;

      EXPECT_LE((expected_cbks - obj.n_cbks), max_misses);
      EXPECT_FALSE(callback_while_tm_null);
      EXPECT_FALSE(callback_while_not_running);

      obj.n_cbks = 0;
    }

    ros::Duration(0.2).sleep();
    for (int it = 0; it < 4; it++)
    {
      auto& obj = objs[it];
      cout << "\tChecking object " << it << std::endl;
      const bool no_callbacks_after_stopped = obj.n_cbks == 0;
      EXPECT_TRUE(no_callbacks_after_stopped);
    }
  }

  {
    cout << "\tTesting callback delay\n";
    for (auto& obj : objs)
    {
      std::scoped_lock lck(obj.mtx);
      obj.check_dt_err = true;
    }
    const ros::Time start = ros::Time::now();
    tom->startAll(start);
    const ros::Duration test_dur(0.5);
    while (ros::Time::now() - start < test_dur)
    {
      ros::Duration(0.0001).sleep();
      ros::spinOnce();
    }
    tom->pauseAll();

    for (int it = 0; it < 4; it++)
    {
      auto& obj = objs[it];
      auto& to = tos[it];
      cout << "\tChecking object " << it << " (timeout: " << to << "s)" << std::endl;
      EXPECT_FALSE(obj.cbk_running);
      std::scoped_lock lck(obj.mtx);

      const bool callback_while_not_running = obj.cbk_not_running;
      const bool callback_while_tm_null = obj.null_cbk;
      cout << "Max. dt error:       " << obj.max_dt_err << "s" << std::endl;
      cout << "Mean dt error:       " << obj.avg_dt_err << "s" << std::endl;
      cout << "Max. expected error: " << obj.max_expected_dt_err << "s" << std::endl;

      EXPECT_LE(obj.max_dt_err, obj.max_expected_dt_err);
      EXPECT_LE(obj.avg_dt_err, update_period);
      EXPECT_FALSE(callback_while_tm_null);
      EXPECT_FALSE(callback_while_not_running);

      obj.n_cbks = 0;
    }

    ros::Duration(0.2).sleep();
    for (int it = 0; it < 4; it++)
    {
      auto& obj = objs[it];
      cout << "\tChecking object " << it << std::endl;
      const bool no_callbacks_after_stopped = obj.n_cbks == 0;
      EXPECT_TRUE(no_callbacks_after_stopped);
    }
  }

  {
    cout << "\tTesting stop from callback\n";
    for (auto& obj : objs)
    {
      std::scoped_lock lck(obj.mtx);
      obj.check_dt_err = true;
    }
    test_stop_from_cbk = true;
    tom->startAll();
    // wait for one callback to be called
    int min_n_cbks = std::numeric_limits<int>::max();
    do
    {
      ros::Rate(1000.0).sleep();
      ros::spinOnce();
      min_n_cbks = std::numeric_limits<int>::max();
      for (const auto& obj : objs)
        min_n_cbks = std::min(min_n_cbks, obj.n_cbks);
    } while (min_n_cbks == 0);

    // wait for the callback to end
    for (auto& obj : objs)
    {
      EXPECT_FALSE(tom->started(obj.timeout_id));
      std::scoped_lock lck(obj.mtx);
      obj.n_cbks = 0;
    }

    ros::Duration max_period(0);
    for (const auto& to : tos)
      max_period = std::max(max_period, to);
    ros::Duration(2*max_period.toSec()).sleep();

    // check that no callbacks were called afterwards
    for (auto& obj : objs)
      EXPECT_EQ(obj.n_cbks, 0);
  }

  {
    cout << "\tTesting destructor\n";
    const ros::Time start = ros::Time::now();
    tom->startAll();
    tom = nullptr;
    const ros::Time destroyed = ros::Time::now();

    for (const auto& obj : objs)
    {
      EXPECT_FALSE(obj.cbk_running);
      EXPECT_FALSE(obj.cbk_not_running);
      EXPECT_FALSE(obj.null_cbk);
    }
    EXPECT_LE((destroyed - start).toSec(), 1.0);
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "TimerTest");
  nh = std::make_unique<ros::NodeHandle>("~");

  ros::Time::waitForValid();

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

