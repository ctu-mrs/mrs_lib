#include "mrs_lib/LKFRepredictor.h"

namespace mrs_lib
{
  /* Model_mrs_odom constructor //{ */
  Model_mrs_odom::Model_mrs_odom(const std::vector<Model_mrs_odom::H_t>& Hs, const Model_mrs_odom::Q_t& Q, const double p1, const double p2, const double p3, const double default_dt)
    : p1(p1), p2(p2), p3(p3)
  {
    Base_class::Q = Q;
    Base_class::Hs = Hs;
    const double& dt = default_dt;
    Base_class::A << 1, dt, 0.5*dt*dt, 0, 0, 0,
                     0, 1, dt, 0, 0, 0,
                     0, 0, 0, 1, 1, 0,
                     0, 0, 0, 0, 0, p3,
                     0, 0, 0, 0, 1, 0,
                     0, 0, 0, 0, 0, p1;
    Base_class::B << 0, 0, 0, 0, 0, dt*p2;
    Base_class::H = Hs.at(0);
  };
  //}

  /* Model_mrs_odom::predict() method //{ */
  Model_mrs_odom::statecov_t Model_mrs_odom::predict(const statecov_t& sc, const u_t& u, double dt, [[maybe_unused]] int param) const
  {
    statecov_t ret;
    ret.x = state_predict_optimized(sc.x, u, dt);
    ret.P = covariance_predict_optimized(sc.P, Base_class::Q, dt);
    /* ret.x = Base_class::state_predict(A, sc.x, B, u); */
    /* ret.P = Base_class::covariance_predict(Base_class::A, sc.P, Base_class::Q); */
    return ret;
  };
  //}

  /* Model_mrs_odom::correct() method //{ */
  Model_mrs_odom::statecov_t Model_mrs_odom::correct(const Model_mrs_odom::statecov_t& sc, const Model_mrs_odom::z_t& z, const Model_mrs_odom::R_t& R, int param) const
  {
    return correction_impl(sc, z, R, Hs.at(param));
  };
  //}

  /* Model_mrs_odom::state_predict_optimized() method //{ */
  Model_mrs_odom::x_t Model_mrs_odom::state_predict_optimized(const Model_mrs_odom::x_t& x_prev, const Model_mrs_odom::u_t& u, double dt) const
  {
    x_t ret = x_prev;
    double& x = ret(0);
    double& dx = ret(1);
    double& ddx = ret(2);
    double& ddxu = ret(3);
    double& ddxd = ret(4);
    double& th = ret(5);
    x = x + dt*dx + 0.5*dt*dt*ddx;
    dx = dx + dt*ddx;
    ddx = ddxu + ddxd;
    ddxu = p3*th;
    /* ddxd = ddxd; */
    th = p1*th + dt*p2*u(0);
    return ret;
  };
  //}

  /* Model_mrs_odom::covariance_predict_optimized() method //{ */
  Model_mrs_odom::P_t Model_mrs_odom::covariance_predict_optimized(const Model_mrs_odom::P_t& P, const Model_mrs_odom::Q_t& Q, double dt) const
  {
    P_t P_ret;
    const double &P11 = P(0, 0), &P12 = P(0, 1), &P13 = P(0, 2), &P14 = P(0, 3), &P15 = P(0, 4), &P16 = P(0, 5);
    const double &P21 = P(1, 0), &P22 = P(1, 1), &P23 = P(1, 2), &P24 = P(1, 3), &P25 = P(1, 4), &P26 = P(1, 5);
    const double &P31 = P(2, 0), &P32 = P(2, 1), &P33 = P(2, 2), &P34 = P(2, 3), &P35 = P(2, 4), &P36 = P(2, 5);
    const double &P41 = P(3, 0), &P42 = P(3, 1), &P43 = P(3, 2), &P44 = P(3, 3), &P45 = P(3, 4), &P46 = P(3, 5);
    const double &P51 = P(4, 0), &P52 = P(4, 1), &P53 = P(4, 2), &P54 = P(4, 3), &P55 = P(4, 4), &P56 = P(4, 5);
    const double &P61 = P(5, 0), &P62 = P(5, 1), &P63 = P(5, 2), &P64 = P(5, 3), &P65 = P(5, 4), &P66 = P(5, 5);
    double dtsqhalf = dt * dt / 2.0;
    P_ret(0, 0) = P11 + P21 * dt + P31 * dtsqhalf + (P12 + P22 * dt + P32 * dtsqhalf) * (dt) + (P13 + P23 * dt + P33 * dtsqhalf) * (dtsqhalf) + Q(0, 0);
    P_ret(0, 1) = P12 + P22 * dt + P32 * dtsqhalf + (P13 + P23 * dt + P33 * dtsqhalf) * (dt) + Q(0, 1);
    P_ret(0, 2) = P14 + P15 + P24 * dt + P25 * dt + P34 * dtsqhalf + P35 * dtsqhalf + Q(0, 2);
    P_ret(0, 3) = (P16 + P26 * dt + P36 * dtsqhalf) * (p3) + Q(0, 3);
    P_ret(0, 4) = P15 + P25 * dt + P35 * dtsqhalf + Q(0, 4);
    P_ret(0, 5) = (P16 + P26 * dt + P36 * dtsqhalf) * (p1) + Q(0, 5);
    P_ret(1, 0) = P21 + P31 * dt + (P22 + P32 * dt) * (dt) + (P23 + P33 * dt) * (dtsqhalf) + Q(1, 0);
    P_ret(1, 1) = P22 + P32 * dt + (P23 + P33 * dt) * (dt) + Q(1, 1);
    P_ret(1, 2) = P24 + P25 + P34 * dt + P35 * dt + Q(1, 2);
    P_ret(1, 3) = (P26 + P36 * dt) * (p3) + Q(1, 3);
    P_ret(1, 4) = P25 + P35 * dt + Q(1, 4);
    P_ret(1, 5) = (P26 + P36 * dt) * (p1) + Q(1, 5);
    P_ret(2, 0) = P41 + P51 + (P42 + P52) * (dt) + (P43 + P53) * (dtsqhalf) + Q(2, 0);
    P_ret(2, 1) = P42 + P52 + (P43 + P53) * (dt) + Q(2, 1);
    P_ret(2, 2) = P44 + P45 + P54 + P55 + Q(2, 2);
    P_ret(2, 3) = (P46 + P56) * (p3) + Q(2, 3);
    P_ret(2, 4) = P45 + P55 + Q(2, 4);
    P_ret(2, 5) = (P46 + P56) * (p1) + Q(2, 5);
    P_ret(3, 0) = p3 * (P61 + P62 * (dt) + P63 * (dtsqhalf)) + Q(3, 0);
    P_ret(3, 1) = p3 * (P62 + P63 * (dt)) + Q(3, 1);
    P_ret(3, 2) = p3 * (P64 + P65) + Q(3, 2);
    P_ret(3, 3) = P66 * p3 * (p3) + Q(3, 3);
    P_ret(3, 4) = P65 * p3 + Q(3, 4);
    P_ret(3, 5) = P66 * p3 * (p1) + Q(3, 5);
    P_ret(4, 0) = P51 + P52 * (dt) + P53 * (dtsqhalf) + Q(4, 0);
    P_ret(4, 1) = P52 + P53 * (dt) + Q(4, 1);
    P_ret(4, 2) = P54 + P55 + Q(4, 2);
    P_ret(4, 3) = P56 * (p3) + Q(4, 3);
    P_ret(4, 4) = P55 + Q(4, 4);
    P_ret(4, 5) = P56 * (p1) + Q(4, 5);
    P_ret(5, 0) = p1 * (P61 + P62 * (dt) + P63 * (dtsqhalf)) + Q(5, 0);
    P_ret(5, 1) = p1 * (P62 + P63 * (dt)) + Q(5, 1);
    P_ret(5, 2) = p1 * (P64 + P65) + Q(5, 2);
    P_ret(5, 3) = P66 * p1 * (p3) + Q(5, 3);
    P_ret(5, 4) = P65 * p1 + Q(5, 4);
    P_ret(5, 5) = P66 * p1 * (p1) + Q(5, 5);
    return P_ret;
  };
  //}

}
