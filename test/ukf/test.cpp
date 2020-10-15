#include <mrs_lib/ukf.h>
#include <mrs_lib/lkf.h>
#include <random>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>
#include <log4cxx/logger.h>

using namespace mrs_lib;
using namespace std;

namespace mrs_lib
{
  const int n_states = 4;
  const int n_inputs = 1;
  const int n_measurements = 2;

  using ukf_t = UKF<n_states, n_inputs, n_measurements>;
  using lkf_t = LKF<n_states, n_inputs, n_measurements>;
}  // namespace mrs_lib

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

H_t H;

// Some helper enums to make the code more readable
enum x_pos
{
  x_x = 0,
  x_y,
  x_alpha,
  x_speed,
};
enum u_pos
{
  u_alpha = 0,
};
enum z_pos
{
  z_x = 0,
  z_y,
};

// This function implements the state transition
x_t tra_model_f(const x_t& x, const u_t& u, const double dt)
{
  x_t ret;
  ret(x_x) = x(x_x) + dt * std::cos(x(x_alpha)) * x(x_speed);
  ret(x_y) = x(x_y) + dt * std::sin(x(x_alpha)) * x(x_speed);
  ret(x_alpha) = x(x_alpha) + dt * u(u_alpha);
  ret(x_speed) = x(x_speed);
  return ret;
}
Eigen::VectorXd tra_model_f2(Eigen::VectorXd x, Eigen::VectorXd u, const double dt)
{
  x_t ret;
  ret(x_x) = x(x_x) + dt * std::cos(x(x_alpha)) * x(x_speed);
  ret(x_y) = x(x_y) + dt * std::sin(x(x_alpha)) * x(x_speed);
  ret(x_alpha) = x(x_alpha) + dt * u(u_alpha);
  ret(x_speed) = x(x_speed);
  return ret;
}

// This function implements the observation generation from a state
ukf_t::z_t obs_model_f(const ukf_t::x_t& x)
{
  return H * x;
}

template <int rows>
Eigen::Matrix<double, rows, 1> normal_randmat(const Eigen::Matrix<double, rows, rows>& cov)
{
  static std::random_device rd{};
  static std::mt19937 gen{rd()};
  static std::normal_distribution<> d{0, 1};
  Eigen::Matrix<double, rows, 1> ret;
  for (int row = 0; row < rows; row++)
    ret(row, 0) = d(gen);
  return cov * ret;
}

/* TEST(TESTSuite, main_test) //{ */

TEST(TESTSuite, main_test) {

  int result = 1;

  srand(std::time(0));
  const double alpha = 1e-3;
  const double kappa = 0;
  const double beta = 2;
  tra_model_t tra_model(tra_model_f);
  tra_model_t tra_model2(tra_model_f2);
  obs_model_t obs_model(obs_model_f);

  // dt will be constant in this example
  const double dt = 1.0;

  // Initialize the process noise matrix
  Q_t Q;
  Q << 1e-3, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 1e-2, 0, 0, 0, 0, 1e-2;

  // Initialize the measurement noise matrix
  R_t R;
  R << 1e-2, 0, 0, 1e-2;

  // Initialize the observation matrix
  H << 1, 0, 0, 0, 0, 1, 0, 0;

  // Generate initial state and covariance
  x_t x0 = 100.0 * x_t::Random();
  x0(2) = 0.0;
  x0(3) = 10.0;
  P_t P_tmp = P_t::Random();
  const P_t P0 = 10.0 * P_tmp * P_tmp.transpose();
  const ukf_t::statecov_t sc0({x0, P0});

  ukf_t ukf(tra_model, obs_model, alpha, kappa, beta);

  const int n_its = 1e2;
  std::vector<ukf_t::statecov_t> uscs;
  std::vector<ukf_t::statecov_t> uscs2;
  std::vector<lkf_t::statecov_t> scs;

  uscs.reserve(n_its + 1);
  uscs2.reserve(n_its + 1);
  scs.reserve(n_its + 1);

  uscs.push_back(sc0);
  uscs2.push_back(sc0);
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

    // Generate a new observation according to the model and noise parameters
    const z_t z = obs_model_f(sc.x) + normal_randmat(R);

    scs.push_back(sc);
    scs.push_back(sc);
    std::cout << "gt       state:" << std::endl << sc.x.transpose() << std::endl;

    auto usc = uscs.back();
    try
    {
      std::cout << "ukf_new  state:" << std::endl << usc.x.transpose() << std::endl;
      usc = ukf.predict(usc, u, Q, dt);
      std::cout << "ukf_new  predi:" << std::endl << usc.x.transpose() << std::endl;
      uscs.push_back(usc);

      usc = ukf.correct(usc, z, R);
      std::cout << "ukf_new  corre:" << std::endl << usc.x.transpose() << std::endl;
      uscs2.push_back(usc);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR("NEW UKF failed: %s", e.what());
      result *= 0;
    }
  }

  ROS_INFO("[%s]: loop 1 finished", ros::this_node::getName().c_str());

  double x_diff = 0.0;
  double P_diff = 0.0;

  for (int it = 0; it < n_its + 1; it++)
  {
    ROS_INFO("[%s]: it %d", ros::this_node::getName().c_str(), it);

    const auto usc = uscs.at(it);
    const auto usc2 = uscs2.at(it);
    /* const auto lsc = lscs.at(it); */
    const auto cur_x_diff = (usc.x - usc2.x).norm();
    const auto cur_P_diff = (usc.P - usc2.P).norm();
    x_diff += cur_x_diff;
    P_diff += cur_P_diff;
  }

  /* std::cout << "x diff average: " << x_diff / (n_its + 1) << std::endl; */
  /* std::cout << "P diff average: " << P_diff / (n_its + 1) << std::endl; */

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
