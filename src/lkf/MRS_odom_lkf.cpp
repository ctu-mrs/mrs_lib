/* author: Matouš Vrba */

#include <mrs_lib/MRS_odom_lkf.h>

namespace mrs_lib
{

  /* constructor //{ */
  MRS_odom_lkf::MRS_odom_lkf(const std::vector<H_t>& Hs, const Q_t& Q, const double p1, const double p2, const double p3, const double default_dt)
    : m_model(Hs, Q, p1, p2, p3, default_dt), m_last_sc({x_t::Zero(), P_t::Identity()}), m_last_u(u_t::Zero()), m_default_dt(default_dt)
  {}
  //}

  /* setters //{ */
  // set new measurement and its covariance
  void MRS_odom_lkf::setMeasurement(const z_t& z, const R_t& R)
  {
    m_last_z = z;
    m_last_R = R;
  }

  // set new measurement
  void MRS_odom_lkf::setMeasurement(const z_t& z)
  {
    m_last_z = z;
  }

  // set new input vector
  void MRS_odom_lkf::setInput(const u_t& u)
  {
    m_last_u = u;
  }

  // set process noise
  void MRS_odom_lkf::setQ(const Q_t& Q)
  {
    m_model.Q = Q;
  }

  // set covariance
  void MRS_odom_lkf::setCovariance(const P_t& P)
  {
    m_last_sc.P = P;
  }

  // set n-th states of the estimate state vector
  void MRS_odom_lkf::setState(const unsigned num, const double value)
  {
    m_last_sc.x(num) = value;
  }

  // set all states of the estimate state vector
  void MRS_odom_lkf::setStates(const x_t& x)
  {
    m_last_sc.x = x;
  }
  //}

  /* getters //{ */
  // return estimated states
  MRS_odom_lkf::x_t MRS_odom_lkf::getStates() const
  {
    return m_last_x;
  }

  // return n-th states of the estimate state vector
  double MRS_odom_lkf::getState(const unsigned num) const
  {
    return m_last_x(num);
  }

  // get the covariance matrix
  MRS_odom_lkf::P_t MRS_odom_lkf::getCovariance() const
  {
    return m_last_P;
  }
  //}

  // do iteration of the filter
  MRS_odom_lkf::statecov_t MRS_odom_lkf::iterate(const double dt, const int H_idx)
  {
    m_last_sc = m_model.predict(m_last_sc, m_last_u, dt);
    m_last_sc = m_model.correct(m_last_sc, m_last_z, m_last_R, H_idx);
    return m_last_sc;
  }
  MRS_odom_lkf::statecov_t MRS_odom_lkf::iterate(const int H_idx)
  {
    return iterate(m_default_dt, H_idx);
  }

  // iterate without the correction phase
  MRS_odom_lkf::statecov_t MRS_odom_lkf::iterateWithoutCorrection(double dt)
  {
    m_last_sc = m_model.predict(m_last_sc, m_last_u, dt);
    return m_last_sc;
  }
  MRS_odom_lkf::statecov_t MRS_odom_lkf::iterateWithoutCorrection()
  {
    return iterateWithoutCorrection(m_default_dt);
  }

  // do just the correction
  MRS_odom_lkf::statecov_t MRS_odom_lkf::doCorrection(const int H_idx)
  {
    m_last_sc = m_model.correct(m_last_sc, m_last_z, m_last_R, H_idx);
    return m_last_sc;
  }


}  // namespace mrs_lib

