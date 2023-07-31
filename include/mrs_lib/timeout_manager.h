// clang: MatousFormat
/**  \file
     \brief TODO
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef TIMEOUT_MANAGER_H
#define TIMEOUT_MANAGER_H

#include <ros/ros.h>

namespace mrs_lib
{
  /* TimeoutManager class //{ */
  /**
  * \brief TODO
  *
  */
  class TimeoutManager
  {
    public:
      // | ---------------------- public types ---------------------- |
      using timeout_id_t = size_t;
      using callback_t = std::function<void(const ros::Time&)>;

    public:
      // | --------------------- public methods --------------------- |

    /*!
      * \brief TODO
      *
      */
      TimeoutManager(const ros::NodeHandle& nh, const ros::Rate& update_rate);

      timeout_id_t registerNew(const ros::Duration& timeout, const callback_t& callback, const ros::Time& last_update = ros::Time::now());

      void reset(const timeout_id_t id, const ros::Time& time = ros::Time::now());

      void pause(const timeout_id_t id);

      void start(const timeout_id_t id, const ros::Time& time = ros::Time::now());

      void change(const timeout_id_t id, const ros::Duration& timeout, const callback_t& callback, const ros::Time& last_update = ros::Time::now());

      ros::Time lastReset(const timeout_id_t id);

      /* implementation details //{ */
      
      private:
        // | ---------------------- private types --------------------- |
        struct timeout_info_t
        {
          bool paused;
          callback_t callback;
          ros::Duration timeout;
          ros::Time last_reset;
          ros::Time last_callback;
        };
      
      private:
        // | --------------------- private methods -------------------- |
        void main_timer_callback([[maybe_unused]] const ros::TimerEvent& evt);
      
      private:
        // | ------------------------- members ------------------------ |
        std::recursive_mutex m_mtx;
        timeout_id_t m_last_id;
        std::vector<timeout_info_t> m_timeouts;
      
        ros::Timer m_main_timer;
      
      //}

  };
  //}
}

#endif // TIMEOUT_MANAGER_H
