// clang: MatousFormat

/**  \file
     \brief Tests for the UKF implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz (rewrite, documentation)
 */

#include <mrs_lib/ukf.h>
#include <mrs_lib/LKFSystemModels.h>

namespace mrs_lib
{
  const int n_states = 2;
  const int n_inputs = 2;
  const int n_measurements = 2;

  using ukf_t = UKF<n_states, n_inputs, n_measurements>;
  using lkf_t = Model_lkf<n_states, n_inputs, n_measurements>;
}

using namespace mrs_lib;
using Q_t = ukf_t::Q_t;
using tra_model_t = ukf_t::transition_model_t;
using obs_model_t = ukf_t::observation_model_t;
using x_t = ukf_t::x_t;
using P_t = ukf_t::P_t;
using u_t = ukf_t::u_t;
using z_t = ukf_t::z_t;
using R_t = ukf_t::R_t;

using A_t = lkf_t::A_t;
using B_t = lkf_t::B_t;
using H_t = lkf_t::H_t;

ukf_t::x_t tra_model_f(const ukf_t::x_t& x, const ukf_t::u_t& u, const double dt)
{
  ukf_t::x_t ret;
  ret(0) = x(0) + x(1)*dt + u(0)*dt;
  ret(1) = x(1) + u(1)*dt;
  return ret;
}

ukf_t::z_t obs_model_f(const ukf_t::x_t& x)
{
  ukf_t::z_t ret = x;
  return ret;
}

int main()
{
  const double alpha = 1e-3;
  const double kappa = 1;
  const double beta  = 2;
  Q_t Q = Q_t::Identity();
  tra_model_t tra_model(tra_model_f);
  obs_model_t obs_model(obs_model_f);
  const double dt = 1.0;

  ukf_t ukf(alpha, kappa, beta, Q, tra_model, obs_model);

  A_t A; A <<
    1, dt,
    0, 1;
  B_t B; B << 
    dt, 0,
    0, dt;
  H_t H; H << 
    1, 0, 
    0, 1;
  lkf_t lkf(A, B, H, Q);

  const ukf_t::statecov_t sc0({
      x_t(0.0, 100.0),
      P_t::Identity()
      });
  const u_t u(1.0, -10.0);

  const ukf_t::statecov_t usc1_0 = ukf.predict(sc0, u, dt);
  const lkf_t::statecov_t lsc1_0 = lkf.predict(sc0, u, dt);

  z_t z; z << 1, 50;
  R_t R = R_t::Identity();

  const ukf_t::statecov_t usc1_1 = ukf.correct(usc1_0, z, R);
  const lkf_t::statecov_t lsc1_1 = lkf.correct(lsc1_0, z, R);

  std::cout << "x[0]:" << std::endl << sc0.x << std::endl;
  std::cout << "P[0]:" << std::endl << sc0.P << std::endl;

  std::cout << "----------------------------------------" << std::endl;

  std::cout << "x[1,0] (UKF):" << std::endl << usc1_0.x << std::endl;
  std::cout << "P[1,0] (UKF):" << std::endl << usc1_0.P << std::endl;
  std::cout << "x[1,1] (UKF):" << std::endl << usc1_1.x << std::endl;
  std::cout << "P[1,1] (UKF):" << std::endl << usc1_1.P << std::endl;

  std::cout << "----------------------------------------" << std::endl;

  std::cout << "x[1,0] (LKF):" << std::endl << lsc1_0.x << std::endl;
  std::cout << "P[1,0] (LKF):" << std::endl << lsc1_0.P << std::endl;
  std::cout << "x[1,1] (LKF):" << std::endl << lsc1_1.x << std::endl;
  std::cout << "P[1,1] (LKF):" << std::endl << lsc1_1.P << std::endl;

  return 0;
}
