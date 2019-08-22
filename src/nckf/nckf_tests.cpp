// clang: MatousFormat

/**  \file
     \brief Tests for the UKF implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#include <mrs_lib/nckf.h>
#include <random>

namespace mrs_lib
{
  const int n_states = 4;
  const int n_states_norm_constrained = 2;
  const int n_inputs = 1;
  const int n_measurements = 2;

  /* using ukf_t = NCUKF<n_states, n_inputs, n_measurements>; */
  using lkf_t = NCLKF_partial<n_states, n_inputs, n_measurements, n_states_norm_constrained>;
}

using namespace mrs_lib;
using Q_t = lkf_t::Q_t;
using x_t = lkf_t::x_t;
using P_t = lkf_t::P_t;
using u_t = lkf_t::u_t;
using z_t = lkf_t::z_t;
using R_t = lkf_t::R_t;

using A_t = lkf_t::A_t;
using B_t = lkf_t::B_t;
using H_t = lkf_t::H_t;

A_t A;
B_t B;
H_t H;


// This function implements the state transition
x_t tra_model_f(const x_t& x, const u_t& u, [[maybe_unused]] const double dt)
{
  x_t ret;
  ret = A*x + B*u;
  auto xq = ret.block<2, 1>(0, 0);
  xq = xq/xq.norm();
  ret.block<2, 1>(2, 0) = xq;
  return ret;
}

// This function implements the observation generation from a state
z_t obs_model_f(const x_t& x)
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
  const double l = 1.0;

  // dt will be constant in this example
  const double dt = 1.0;

  // Initialize the process noise matrix
  Q_t Q; Q << 
    1e-3, 0, 0, 0,
    0, 1e-3, 0, 0,
    0, 0, 1e-2, 0,
    0, 0, 0, 1e-2;

  // Initialize the measurement noise matrix
  R_t R; R <<
    1e-2, 0,
    0, 1e-2;

  // Initialize the state transition matrix
  A << 
    1, dt, 0, 0,
    0, 1, dt, 0,
    0, 0, 1, dt,
    0, 0, 0, 0.9;

  // Initialize the input matrix
  B << 
    0,
    0,
    0,
    1;

  // Initialize the observation matrix
  H <<
    1, 0, 0, 0,
    0, 1, 0, 0;

  // Generate initial state and covariance
  x_t x0 = 100.0*x_t::Random();
  x0(2) = 0.0;
  x0(3) = 10.0;
  x0 = x0/x0.norm();
  P_t P_tmp = P_t::Random();
  const P_t P0 = 10.0*P_tmp*P_tmp.transpose();
  const lkf_t::statecov_t sc0({x0, P0});

  lkf_t lkf(A, B, H, l, {0, 1});

  const int n_its = 1e1;
  std::vector<lkf_t::statecov_t> lscs;
  std::vector<lkf_t::statecov_t> scs;

  lscs.reserve(n_its+1);
  scs.reserve(n_its+1);

  lscs.push_back(sc0);
  scs.push_back(sc0);

  for (int it = 0; it < n_its; it++)
  {
    std::cout << "step: " << it << std::endl;

    // Generate a new input vector
    /* const u_t u = u_t(0.5) + 0.1*u_t::Random(); */
    const u_t u = u_t(0.0);
    
    // Generate a new state according to the model and noise parameters
    auto sc = scs.back();
    sc.x = tra_model_f(sc.x, u, dt) + normal_randmat(Q);
    sc.x = sc.x / sc.x.norm();
    
    // Generate a new observation according to the model and noise parameters
    const z_t z = obs_model_f(sc.x) + normal_randmat(R);

    scs.push_back(sc);
    scs.push_back(sc);
    std::cout << "gt       state:" << std::endl << sc.x.transpose() << std::endl;

    {
      auto lsc = lscs.back();
      try
      {
        std::cout << "lkf_new  state:" << std::endl << lsc.x.transpose() << std::endl;
        lsc = lkf.predict(lsc, u, Q, dt);
        std::cout << "lkf_new  predi:" << std::endl << lsc.x.transpose() << std::endl;
        lscs.push_back(lsc);
        lsc = lkf.correct(lsc, z, R);
        std::cout << "lkf_new  corre:" << std::endl << lsc.x.transpose() << std::endl;
        std::cout << "lkf_new  corre norm:" << lsc.x.block<2, 1>(0, 0).norm() << std::endl;
        lscs.push_back(lsc);
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("NEW  LKF failed: %s", e.what());
      }
    }
  }

  double x_diff = 0.0;
  double P_diff = 0.0;
  for (int it = 0; it < n_its; it++)
  {
    const auto lsc = lscs.at(it);
    const auto cur_x_diff = (lsc.x-lsc.x).norm();
    const auto cur_P_diff = (lsc.P-lsc.P).norm();
    x_diff += cur_x_diff;
    P_diff += cur_P_diff;
  }

  std::cout << "x diff average: " << x_diff/(n_its+1) << std::endl;
  std::cout << "P diff average: " << P_diff/(n_its+1) << std::endl;

  return 0;
}

