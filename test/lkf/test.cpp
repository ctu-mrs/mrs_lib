#include <gtest/gtest.h>

#include <mrs_lib/mutex.h>

// Include the LKF header
#include <mrs_lib/lkf.h>
#include <random>

// Define the LKF we will be using
namespace mrs_lib
{
  const int n_states = 4;
  const int n_inputs = 2;
  const int n_measurements = 2;

  using lkf_t = LKF<n_states, n_inputs, n_measurements>;
}  // namespace mrs_lib

// Some helpful aliases to make writing of types shorter
using namespace mrs_lib;
using A_t = lkf_t::A_t;
using B_t = lkf_t::B_t;
using H_t = lkf_t::H_t;
using Q_t = lkf_t::Q_t;
using x_t = lkf_t::x_t;
using P_t = lkf_t::P_t;
using u_t = lkf_t::u_t;
using z_t = lkf_t::z_t;
using R_t = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

// Some helper enums to make the code more readable
enum x_pos
{
  x_x = 0,
  x_y = 1,
  x_dx = 2,
  x_dy = 3,
};
enum u_pos
{
  u_dx = 0,
  u_dy = 0,
};
enum z_pos
{
  z_x = 0,
  z_y = 1,
};

A_t A;
B_t B;
H_t H;

x_t tra_model_f(const x_t& x, const u_t& u, [[maybe_unused]] const double dt)
{
  return A * x + B * u;
}

// This function implements the observation generation from a state
z_t obs_model_f(const x_t& x)
{
  return H * x;
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

TEST(TESTSuite, set_mutexed_single) {

  // dt will be constant in this example
  const double dt = 1.0;

  // Initialize the state transition matrix
  A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

  // Initialize the input to state mapping matrix
  B << 0, 0, 0, 0, 1, 0, 0, 1;

  // Initialize the state to measurement mapping matrix
  H << 1, 0, 0, 0, 0, 1, 0, 0;

  // Initialize the process noise matrix
  Q_t Q;
  Q << 1e-3, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 1e-2, 0, 0, 0, 0, 1e-2;

  // Initialize the measurement noise matrix
  R_t R;
  R << 1e-2, 0, 0, 1e-2;

  // Generate initial state and covariance
  const x_t x0 = 100.0 * x_t::Random();
  P_t P_tmp = P_t::Random();
  const P_t P0 = 10.0 * P_tmp * P_tmp.transpose();
  const statecov_t sc0({x0, P0});

  // Instantiate the LKF itself
  lkf_t lkf(A, B, H);

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
      sc_est = lkf.predict(sc_est, u, Q, dt);

      // Apply the correction step
      sc_est = lkf.correct(sc_est, z, R);
    }
    catch (const std::exception& e)
    {
      // In case of error, alert the user
      std::cout << "LKF failed" << std::endl;
    }

    const auto error = (x_gt - sc_est.x).norm();

    EXPECT_LE(error, 0.5);

    std::cout << "Current LKF estimation error is: " << error << std::endl;
    std::cout << "Current ground-truth state is:  " << x_gt.transpose() << std::endl;
    std::cout << "Current LKF estimated state is: " << sc_est.x.transpose() << std::endl;
    std::cout << "Current LKF state covariance is:" << std::endl << sc_est.P << std::endl;
  }
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
