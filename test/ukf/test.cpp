#include <gtest/gtest.h>

#include <mrs_lib/mutex.h>

#include <mrs_lib/ukf.h>

#include <random>

// Define the UKF we will be using
namespace mrs_lib
{
  const int n_states = 3;
  const int n_inputs = 1;
  const int n_measurements = 2;

  using ukf_t = UKF<n_states, n_inputs, n_measurements>;
}  // namespace mrs_lib

// Some helpful aliases to make writing of types shorter
using namespace mrs_lib;
using Q_t = ukf_t::Q_t;
using tra_model_t = ukf_t::transition_model_t;
using obs_model_t = ukf_t::observation_model_t;
using x_t = ukf_t::x_t;
using P_t = ukf_t::P_t;
using u_t = ukf_t::u_t;
using z_t = ukf_t::z_t;
using R_t = ukf_t::R_t;
using statecov_t = ukf_t::statecov_t;

// Some helper enums to make the code more readable
enum x_pos
{
  x_x = 0,
  x_y = 1,
  x_alpha = 2,
};
enum u_pos
{
  u_alpha = 0,
};
enum z_pos
{
  z_x = 0,
  z_y = 1,
};

// This function implements the state transition
const double speed = 0.3;
x_t tra_model_f(const x_t& x, const u_t& u, const double dt)
{
  x_t ret;
  ret(x_x) = x(x_x) + dt * std::cos(x(x_alpha)) * speed;
  ret(x_y) = x(x_y) + dt * std::sin(x(x_alpha)) * speed;
  ret(x_alpha) = x(x_alpha) + u(u_alpha);
  return ret;
}

// This function implements the observation generation from a state
ukf_t::z_t obs_model_f(const ukf_t::x_t& x)
{
  z_t ret;
  ret(z_x) = x(x_x);
  ret(z_y) = x(x_y);
  return ret;
}

// Helper function to generate a random Eigen matrix with normal distribution
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

/* TEST(TESTSuite, set_mutexed_single) //{ */

TEST(TESTSuite, set_mutexed_single)
{

  // Parameters of the Unscented Kalman Filter
  const double alpha = 1e-3;
  const double kappa = 1;
  const double beta = 2;
  tra_model_t tra_model(tra_model_f);
  obs_model_t obs_model(obs_model_f);

  // dt will be constant in this example
  const double dt = 1.0;

  // Initialize the process noise matrix
  Q_t Q;
  Q << 1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-2;

  // Initialize the measurement noise matrix
  R_t R;
  R << 1e-2, 0, 0, 1e-2;

  // Generate initial state and covariance
  const x_t x0 = 100.0 * x_t::Random();
  P_t P_tmp = P_t::Random();
  const P_t P0 = 10.0 * P_tmp * P_tmp.transpose();
  const ukf_t::statecov_t sc0({x0, P0});

  // Instantiate the UKF itself
  ukf_t ukf(tra_model, obs_model, alpha, kappa, beta);

  const int n_its = 1e4;
  // Prepare the ground-truth state and the estimated state and covariance
  x_t x_gt = sc0.x;
  statecov_t sc_est = sc0;

  for (int it = 0; it < n_its; it++)
  {
    std::cout << "step: " << it << std::endl;

    // Generate a new input vector
    const u_t u = u_t::Random();

    // Generate a new state according to the model and noise parameters
    x_gt = tra_model_f(x_gt, u, dt) + normal_randmat(Q);

    // Generate a new observation according to the model and noise parameters
    const z_t z = obs_model_f(x_gt) + normal_randmat(R);

    // There should be a try-catch here to prevent program crashes
    // in case of numerical instabilities (which are possible with UKF)
    try
    {
      // Apply the prediction step
      sc_est = ukf.predict(sc_est, u, Q, dt);

      // Apply the correction step
      sc_est = ukf.correct(sc_est, z, R);
    }
    catch (const std::exception& e)
    {
      // In case of error, alert the user
      std::cout << "UKF failed: " << e.what() << std::endl;
    }

    const auto error = (x_gt - sc_est.x).norm();

    std::cout << "Current UKF estimation error is: " << error << std::endl;
    std::cout << "Current ground-truth state is:  " << x_gt.transpose() << std::endl;
    std::cout << "Current UKF estimated state is: " << sc_est.x.transpose() << std::endl;
    std::cout << "Current UKF state covariance is:" << std::endl << sc_est.P << std::endl;

    EXPECT_LE(error, 0.5);
  }
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
