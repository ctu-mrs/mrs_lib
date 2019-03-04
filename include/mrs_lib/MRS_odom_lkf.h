#ifndef MRS_ODOM_LKF_H
#define MRS_ODOM_LKF_H

#include <mutex>
#include "mrs_lib/LKFSystemModels.h"

namespace mrs_lib
{

  /* specialized Lkf declaration for MRS odom //{ */
  class MRS_odom_lkf {
    public:
      /* definitions, declarations etc. //{ */
      static const int n = Model_mrs_odom::n;
      static const int m = Model_mrs_odom::m;
      static const int p = Model_mrs_odom::p;

      using x_t = typename Model_mrs_odom::x_t;
      using u_t = typename Model_mrs_odom::u_t;
      using z_t = typename Model_mrs_odom::z_t;
      using P_t = typename Model_mrs_odom::P_t;
      using R_t = typename Model_mrs_odom::R_t;
      using statecov_t = typename Model_mrs_odom::statecov_t;
      using A_t = typename Model_mrs_odom::A_t;  // measurement mapping p*n
      using B_t = typename Model_mrs_odom::B_t;  // process covariance n*n
      using H_t = typename Model_mrs_odom::H_t;  // measurement mapping p*n
      using Q_t = typename Model_mrs_odom::Q_t;  // process covariance n*n
      //}

    public:
      MRS_odom_lkf(const std::vector<H_t>& Hs, const Q_t& Q, const double p1, const double p2, const double p3, const double default_dt = 1);

      /* setters //{ */
      // set new measurement and its covariance
      void setMeasurement(const z_t& z, const R_t& R);

      // set new measurement
      void setMeasurement(const z_t& z);

      // set new input vector
      void setInput(const u_t& u);

      // set process noise
      void setQ(const Q_t& Q);

      // set covariance
      void setCovariance(const P_t& cov);

      // set n-th states of the estimate state vector
      void setState(const unsigned num, const double value);

      // set all states of the estimate state vector
      void setStates(const x_t& states);
      //}

      /* getters //{ */
      // return estimated states
      x_t getStates() const;

      // return n-th states of the estimate state vector
      double getState(const unsigned num) const;

      // get the covariance matrix
      P_t getCovariance() const;
      //}

      // do iteration of the filter
      statecov_t iterate(double dt, const int H_idx);
      statecov_t iterate(const int H_idx);

      // iterate without the correction phase
      statecov_t iterateWithoutCorrection(double dt);
      statecov_t iterateWithoutCorrection();

      // do just the correction
      statecov_t doCorrection(const int H_idx);

    private:
      Model_mrs_odom m_model;

      statecov_t m_last_sc;
      x_t& m_last_x = m_last_sc.x;    // last state vector
      P_t& m_last_P = m_last_sc.P;    // last state covariance

      z_t m_last_z;    // the last measurement
      R_t m_last_R;    // last measurement covariance

      u_t m_last_u;    // last system input vector

      const double m_default_dt;

      mutable std::mutex lkf_mutex;
  };
  //}

};

#endif // MRS_ODOM_LKF_H
