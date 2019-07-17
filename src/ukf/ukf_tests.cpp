// clang: MatousFormat

/**  \file
     \brief Tests for the UKF implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz (rewrite, documentation)
 */

#include <mrs_lib/ukf.h>
#include <mrs_lib/LKFSystemModels.h>

namespace mrs_lib
{
  const int n_states = 9;
  const int n_inputs = 8;
  const int n_measurements = 7;

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

A_t A;
B_t B;
H_t H;

ukf_t::x_t tra_model_f(const ukf_t::x_t& x, const ukf_t::u_t& u, [[maybe_unused]] const double dt)
{
  return A*x + B*u;
}

ukf_t::z_t obs_model_f(const ukf_t::x_t& x)
{
  return H*x;
}

int main()
{
  srand(std::time(0));
  const double alpha = 1e-3;
  const double kappa = 1;
  const double beta  = 2;
  tra_model_t tra_model(tra_model_f);
  obs_model_t obs_model(obs_model_f);
  const double dt = 1.0;
  const double r = 10.0;
  const Q_t Q_tmp = r*Q_t::Random();
  const Q_t Q = Q_tmp*Q_tmp.transpose();

  A = r*A_t::Random();
  B = r*B_t::Random();
  H = r*H_t::Random();
  const x_t x0 = r*x_t::Random();
  P_t P_tmp = P_t::Random();
  const P_t P0 = r*P_tmp*P_tmp.transpose();
  const ukf_t::statecov_t sc0({x0, P0});

  ukf_t ukf(alpha, kappa, beta, Q, tra_model, obs_model);
  lkf_t lkf(A, B, H, Q);

  const int n_its = 1e3;
  std::vector<ukf_t::statecov_t> uscs;
  std::vector<lkf_t::statecov_t> lscs;
  std::vector<lkf_t::statecov_t> scs;
  uscs.reserve(n_its+1);
  lscs.reserve(n_its+1);
  scs.reserve(n_its+1);
  uscs.push_back(sc0);
  lscs.push_back(sc0);
  scs.push_back(sc0);

  for (int it = 0; it < n_its; it++)
  {
    const Q_t Q_tmp = r*Q_t::Random();
    const Q_t Q = Q_tmp*Q_tmp.transpose();
    const R_t R_tmp = r*R_t::Random();
    const R_t R = R_tmp*R_tmp.transpose();
    const u_t u = r*u_t::Random();
    auto sc = scs.back();
    sc.x = A*sc.x + B*u + Q*x_t::Random();
    const z_t z = H*sc.x + R*z_t::Random();
    scs.push_back(sc);

    lkf.Q = Q;
    auto lsc = lscs.back();
    lsc = lkf.predict(lsc, u, dt);
    lscs.push_back(lsc);
    lsc = lkf.correct(lsc, z, R);
    lscs.push_back(lsc);

    ukf.setQ(Q);
    auto usc = uscs.back();
    usc = ukf.predict(usc, u, dt);
    uscs.push_back(usc);
    usc = ukf.correct(usc, z, R);
    uscs.push_back(usc);
  }

  double x_diff = 0.0;
  double P_diff = 0.0;
  for (int it = 0; it < n_its+1; it++)
  {
    const auto usc = uscs.at(it);
    const auto lsc = lscs.at(it);
    const auto cur_x_diff = (usc.x-lsc.x).norm();
    const auto cur_P_diff = (usc.P-lsc.P).norm();
    x_diff += cur_x_diff;
    P_diff += cur_P_diff;
  }

  std::cout << "x diff average: " << x_diff/(n_its+1) << std::endl;
  std::cout << "P diff average: " << P_diff/(n_its+1) << std::endl;

  return 0;
}
