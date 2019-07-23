// clang: MatousFormat

/**  \file
     \brief Tests for the UKF implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#include <mrs_lib/ukf.h>
#include <mrs_lib/Ukf.h>
#include <mrs_lib/lkf.h>
#include <random>

namespace mrs_lib
{
  const int n_states = 9;
  const int n_inputs = 8;
  const int n_measurements = 7;

  using ukf_t = UKF<n_states, n_inputs, n_measurements>;
  using lkf_t = LKF<n_states, n_inputs, n_measurements>;
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

template class mrs_lib::UKF<n_states, n_inputs, n_measurements>;
template class mrs_lib::LKF<n_states, n_inputs, n_measurements>;

A_t A;
B_t B;
H_t H;

ukf_t::x_t tra_model_f(const ukf_t::x_t& x, const ukf_t::u_t& u, [[maybe_unused]] const double dt)
{
  return A*x + B*u;
}

Eigen::VectorXd tra_model_f2(Eigen::VectorXd x, Eigen::VectorXd u, [[maybe_unused]] const double dt)
{
  return A*x + B*u;
}

ukf_t::z_t obs_model_f(const ukf_t::x_t& x)
{
  return H*x;
}

template <int rows>
Eigen::Matrix<double, rows, 1> normal_randmat(const Eigen::Matrix<double, rows, rows>& cov)
{
    static std::random_device rd{};
    static std::mt19937 gen{rd()};
    static std::normal_distribution<> d{0,1};
    Eigen::Matrix<double, rows, 1> ret;
    for (int row = 0; row < rows; row++)
      ret(row, 0) = d(gen);
    return cov*ret;
}

int main()
{
  srand(std::time(0));
  const double alpha = 1e-3;
  const double kappa = 1;
  const double beta  = 2;
  tra_model_t tra_model(tra_model_f);
  mrs_lib::model tra_model2(tra_model_f2);
  obs_model_t obs_model(obs_model_f);
  const double dt = 1.0;
  const double r = 10.0;
  const Q_t Q = r*Q_t::Identity();
  /* const R_t R = r*R_t::Identity(); */

  A = A_t::Identity();
  B = r*B_t::Random();
  H = r*H_t::Random();
  const x_t x0 = r*x_t::Random();
  P_t P_tmp = P_t::Random();
  const P_t P0 = r*P_tmp*P_tmp.transpose();
  const ukf_t::statecov_t sc0({x0, P0});

  ukf_t ukf(alpha, kappa, beta, Q, tra_model, obs_model);
  const R_t R_tmp = R_t::Random();
  const R_t R = r*R_tmp*R_tmp.transpose();
  mrs_lib::Ukf ukf2(n_states, n_inputs, n_measurements, alpha, kappa, beta, Q, R, H, tra_model2);
  lkf_t lkf(A, B, H, Q);

  const int n_its = 1e4;
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
    std::cout << "step: " << it << std::endl;
    const Q_t Q_tmp = Q_t::Random();
    const Q_t Q = r*Q_tmp*Q_tmp.transpose();
    const R_t R_tmp = R_t::Random();
    const R_t R = r*R_tmp*R_tmp.transpose();
    const u_t u = r*u_t::Random();
    /* const x_t x = r*x_t::Random(); */
    /* const P_t P = r*P_t::Random(); */
    /* const z_t z = r*z_t::Random(); */
    /* ukf_t::statecov_t sc {x, P}; */
    /* scs.push_back(sc); */
    /* scs.push_back(sc); */
    auto sc = scs.back();
    sc.x = tra_model_f(sc.x, u, dt) + normal_randmat(Q);
    scs.push_back(sc);
    const z_t z = obs_model_f(sc.x) + normal_randmat(R);
    scs.push_back(sc);

    lkf.Q = Q;
    auto lsc = lscs.back();
    lsc = lkf.predict(lsc, u, dt);
    lscs.push_back(lsc);
    lsc = lkf.correct(lsc, z, R);
    lscs.push_back(lsc);

    ukf.setQ(Q);
    auto usc = uscs.back();
    try
    {
      usc = ukf.predict(usc, u, dt);
      uscs.push_back(usc);
      usc = ukf.correct(usc, z, R);
      uscs.push_back(usc);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("UKF failed: %s", e.what());
    }

    const auto cur_x_diff = (usc.x-lsc.x).norm();
    const auto cur_P_diff = (usc.P-lsc.P).norm();
    std::cout << "cur. x diff: " << cur_x_diff << std::endl;
    std::cout << "cur. P diff: " << cur_P_diff << std::endl;

    const auto cur_lgt_diff = (sc.x-lsc.x).norm();
    const auto cur_ugt_diff = (sc.x-usc.x).norm();
    std::cout << "cur. lgt diff: " << cur_lgt_diff << std::endl;
    std::cout << "cur. ugt diff: " << cur_ugt_diff << std::endl;
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
