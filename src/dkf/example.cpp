// clang: MatousFormat
/**  \file
     \brief Example file for the DKF implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib dkf_example`.

     See \ref dkf/example.cpp.
 */

/**  \example "dkf/example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib dkf_example`.
 */

// Include the DKF header
#include <mrs_lib/dkf.h>
#include <random>
#include <ros/ros.h>

// Define the KF we will be using
namespace mrs_lib
{
  const int n_states = 6;
  const int n_inputs = 0;

  using dkf_t = DKF<n_states, n_inputs>;
}

// Some helpful aliases to make writing of types shorter
using namespace mrs_lib;
using A_t = dkf_t::A_t;
using B_t = dkf_t::B_t;
using H_t = dkf_t::H_t;
using Q_t = dkf_t::Q_t;
using x_t = dkf_t::x_t;
using P_t = dkf_t::P_t;
using u_t = dkf_t::u_t;
using z_t = dkf_t::z_t;
using R_t = dkf_t::R_t;
using statecov_t = dkf_t::statecov_t;
using pt3_t = dkf_t::pt3_t;
using vec3_t = dkf_t::vec3_t;
using mat3_t = Eigen::Matrix3d;

// Some helper enums to make the code more readable
enum x_pos
{
  x_x = 0,
  x_y = 1,
  x_z = 2,
  x_dx = 3,
  x_dy = 4,
  x_dz = 5,
};

// Helper function to generate a random Eigen matrix with normal distribution
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

A_t A;

// This function implements the state transition
x_t tra_model_f(const x_t& x)
{
  return A*x;
}

struct obs_t
{
  pt3_t line_origin;
  vec3_t line_direction;
  double line_variance;
};

// This function implements the observation generation from a state
obs_t observation(const x_t& x, const double meas_std)
{
  // the first three states are the x, y, z coordinates of the target's position
  const pt3_t target = x.head<3>();
  const vec3_t meas_noise = normal_randmat<3>(meas_std*meas_std*mat3_t::Identity());

  obs_t ret;
  // generate a random observation point
  const pt3_t observer = target + 100.0*pt3_t::Random();
  ret.line_direction = (target - observer).normalized();
  ret.line_origin = observer + meas_noise;
  ret.line_variance = meas_std*meas_std;
  return ret;
}

int main()
{
  // dt will be constant in this example
  const double dt = 1.0;
  const double sigma_a = 0.03;
  const double sigma_R = 0.1;

  // Initialize the state transition matrix
  A = A_t::Identity();
  A.block<3, 3>(0, 3) = dt*mat3_t::Identity();

  const mat3_t Q_a = sigma_a*sigma_a*mat3_t::Identity();
  Eigen::Matrix<double, 6, 3> B_Q;
  B_Q.block<3, 3>(0, 0) = 0.5*dt*dt*mat3_t::Identity();
  B_Q.block<3, 3>(3, 0) = dt*mat3_t::Identity();
  // Initialize the process noise matrix
  const Q_t Q = B_Q * Q_a * B_Q.transpose();

  // Generate initial state and covariance
  x_t x_gt = 100.0*x_t::Random();

  const P_t P_tmp = P_t::Random();
  const P_t P0 = 10.0*P_tmp*P_tmp.transpose();
  const x_t x0 = x_gt + normal_randmat(P0);
  const statecov_t sc0({x0, P0});

  // Instantiate the KF itself
  dkf_t kf(A, B_t::Zero());

  const int n_its = 1e4;
  // Prepare the ground-truth state and the estimated state and covariance
  statecov_t sc_est = sc0;

  std::cout << "A: " << A << "\n";
  std::cout << "Q: " << Q << "\n";
  std::cout << "x0_gt: " << x_gt << "\n";
  std::cout << "x0_est: " << sc_est.x << "\n";

  for (int it = 0; it < n_its; it++)
  {
    /* std::cout << "step: " << it << std::endl; */

    // Generate a new input vector
    const u_t u = u_t::Random();
    
    // Generate a new state according to the model and noise parameters
    x_gt = tra_model_f(x_gt) + normal_randmat(Q);
    
    // Generate a new observation according to the model and noise parameters
    const obs_t obs = observation(x_gt, sigma_R);

    // There should be a try-catch here to prevent program crashes
    // in case of numerical instabilities (which are possible with UKF)
    try
    {
      // Apply the prediction step
      sc_est = kf.predict(sc_est, u, Q, dt);
      
      // Apply the correction step
      sc_est = kf.correctLine(sc_est, obs.line_origin, obs.line_direction, obs.line_variance);
    }
    catch (const std::exception& e)
    {
      // In case of error, alert the user
      ROS_ERROR("KF failed: %s", e.what());
    }

  }

  const x_t error = x_gt - sc_est.x;
  const double RMSE = (error).norm();
  const double maha = std::sqrt( error.transpose() * sc_est.P.inverse() * error );
  std::cout << "Current KF estimation error is: " << RMSE << std::endl;
  std::cout << "Current KF mahalanobis error is: " << maha << std::endl;
  std::cout << "Current ground-truth state is:  " << x_gt.transpose() << std::endl;
  std::cout << "Current KF estimated state is: " << sc_est.x.transpose() << std::endl;
  std::cout << "Current KF state covariance is:" << std::endl << sc_est.P << std::endl;

  return 0;
}


