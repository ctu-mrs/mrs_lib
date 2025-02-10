// clang: MatousFormat

#include <mrs_lib/lkf.h>

namespace mrs_lib
{
  /* LKF_MRS_odom constructor //{ */
  LKF_MRS_odom::LKF_MRS_odom(const std::vector<LKF_MRS_odom::H_t>& Hs, const double default_dt)
    : m_Hs(Hs)
  {
    const double& dt = default_dt;
    Base_class::A << 1, dt, 0.5 * dt * dt, 0, 1, dt, 0, 0, 0.9;
    Base_class::B << 0, 0, 0.1;
  }
  //}

  /* LKF_MRS_odom::predict() method //{ */
  LKF_MRS_odom::statecov_t LKF_MRS_odom::predict(const statecov_t& sc, const u_t& u, const Q_t& Q, double dt) const
  {
    statecov_t ret;
    ret.x = state_predict_optimized(sc.x, u, dt);
    ret.P = covariance_predict_optimized(sc.P, Q, dt);
    /* ret.x = Base_class::state_predict(A, sc.x, B, u); */
    /* ret.P = Base_class::covariance_predict(Base_class::A, sc.P, Base_class::Q); */
    return ret;
  }
  //}

  /* LKF_MRS_odom::correct() method //{ */
  LKF_MRS_odom::statecov_t LKF_MRS_odom::correct(const LKF_MRS_odom::statecov_t& sc, const LKF_MRS_odom::z_t& z, const LKF_MRS_odom::R_t& R, int param) const
  {
    /* return correction_impl(sc, z, R, Hs.at(param)); */
    return correction_optimized(sc, z, R, m_Hs.at(param));
  }
  //}

  /* LKF_MRS_odom::state_predict_optimized() method //{ */
  LKF_MRS_odom::x_t LKF_MRS_odom::state_predict_optimized(const LKF_MRS_odom::x_t& x_prev, const LKF_MRS_odom::u_t& u, double dt) const
  {
    x_t ret = x_prev;
    double& x = ret(0);
    double& dx = ret(1);
    double& ddx = ret(2);
    x = x + dt * dx + 0.5 * dt * dt * ddx;
    dx = dx + dt * ddx;
    ddx = 0.9 * ddx + 0.1 * u(0);
    return ret;
  }
  //}

  /* LKF_MRS_odom::covariance_predict_optimized() method //{ */
  LKF_MRS_odom::P_t LKF_MRS_odom::covariance_predict_optimized(const LKF_MRS_odom::P_t& P, const LKF_MRS_odom::Q_t& Q, double dt) const
  {
    P_t P_ret;
    const double &P11 = P(0, 0), &P12 = P(0, 1), &P13 = P(0, 2);
    const double &P21 = P(1, 0), &P22 = P(1, 1), &P23 = P(1, 2);
    const double &P31 = P(2, 0), &P32 = P(2, 1), &P33 = P(2, 2);
    const double dtsqhalf = dt * dt / 2.0;
    P_ret(0, 0) = P11 + P21 * dt + P31 * dtsqhalf + (P12 + P22 * dt + P32 * dtsqhalf) * (dt) + (P13 + P23 * dt + P33 * dtsqhalf) * (dtsqhalf) + Q(0, 0);
    P_ret(0, 1) = P12 + P22 * dt + P32 * dtsqhalf + (P13 + P23 * dt + P33 * dtsqhalf) * (dt) + Q(0, 1);
    P_ret(0, 2) = P13 + P23 * dt + P33 * dtsqhalf + Q(0, 2);
    P_ret(1, 0) = P21 + P31 * dt + (P22 + P32 * dt) * (dt) + (P23 + P33 * dt) * (dtsqhalf) + Q(1, 0);
    P_ret(1, 1) = P22 + P32 * dt + (P23 + P33 * dt) * (dt) + Q(1, 1);
    P_ret(1, 2) = P23 + P33 * dt + Q(1, 2);
    P_ret(2, 0) = P31 + P32 * (dt) + P33 * (dtsqhalf) + Q(2, 0);
    P_ret(2, 1) = P32 + P33 * (dt) + Q(2, 1);
    P_ret(2, 2) = P33 + Q(2, 2);
    return P_ret;
  }
  //}

}  // namespace mrs_lib
