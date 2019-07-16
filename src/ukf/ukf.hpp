/* author: Tomas Baca */

#include <ros/ros.h>
#include <mrs_lib/ukf.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

namespace mrs_lib
{

  template <int n_states, int n_inputs, int n_measurements>
  using statecov_t = typename UKF<n_states, n_inputs, n_measurements>::statecov_t;

  template <int n_states, int n_inputs, int n_measurements>
  using X_t = typename UKF<n_states, n_inputs, n_measurements>::X_t;

  template <int n_states, int n_inputs, int n_measurements>
  using P_t = typename UKF<n_states, n_inputs, n_measurements>::P_t;

/* constructor //{ */

template <int n_states, int n_inputs, int n_measurements>
UKF<n_states, n_inputs, n_measurements>::UKF()
{
}

template <int n_states, int n_inputs, int n_measurements>
UKF<n_states, n_inputs, n_measurements>::UKF(const double alpha, const double k, const double beta, const R_t& R, const Q_t& Q, const H_t& H, const model_t& model)
  : m_alpha(alpha), m_k(k), m_beta(beta), m_Wm(W_t::Zeros()), m_Wc(W_t::Zeros()), m_R(R), m_Q(Q), m_H(H), m_model(model)
{
  computeWeights();
}

//}

/* computeWeights() //{ */

template <int n_states, int n_inputs, int n_measurements>
void UKF<n_states, n_inputs, n_measurements>::computeWeights()
{
  // initialize lambda
  m_lambda = double(n) * (m_alpha * m_alpha - 1.0);

  // initialize first terms of the weights
  m_Wm(0) = m_lambda / (double(n) + m_lambda);
  m_Wc(0) = m_Wm(0) + (1.0 - m_alpha * m_alpha + m_beta);

  // initialize the rest of the weights
  for (int i = 1; i < w; i++)
  {
    m_Wm(i) = 1.0 / (2.0 * (double(n) + m_lambda));
    m_Wc(i) = m_Wm(i);
  }
}

//}

/* setConstants() //{ */

template <int n_states, int n_inputs, int n_measurements>
// update the UKF constants
void UKF<n_states, n_inputs, n_measurements>::setConstants(const double alpha, const double k, const double beta)
{
  m_alpha = alpha;
  m_k     = k;
  m_beta  = beta;

  computeWeights();
}

//}

/* computeSigmas() method //{ */
template <int n_states, int n_inputs, int n_measurements>
X_t<n_states, n_inputs, n_measurements> UKF<n_states, n_inputs, n_measurements>::computeSigmas(const x_t& x, const P_t& P_sqrt) const
{
  // calculate sigma points
  // fill in the middle of the elipsoid
  X_t X;
  X.col(0) = x;

  // positive sigma points
  for (int i = 1; i <= n; i++)
  {
    X.col(i) = x + P_sqrt.row(i - 1).transpose();
  }

  // negative sigma points
  for (int i = n+1; i <= 2*n; i++)
  {
    X.col(i) = x - P_sqrt.row(i - n - 1).transpose();
  }
}
//}

/* computePSqrt() method //{ */
template <int n_states, int n_inputs, int n_measurements>
P_t<n_states, n_inputs, n_measurements> UKF<n_states, n_inputs, n_measurements>::computePSqrt(const P_t& P) const
{
  // calculate the square root of the covariance matrix
  P_t P_sqrt = (double(n) + m_lambda) * P;
  const P_t P_debug = P_sqrt;

  // check if matrix is regular
  Eigen::EigenSolver<P_t> es(P_sqrt);
  if ((es.eigenvalues().array().real() <= 0.0).any())
  {
    ROS_ERROR("UKF: Some eigenvalues of P_temp are <= 0!");
    throw square_root_exception();
  }

  // if there are problems with square rooting P, try this:
  /* P_temp = 0.5*P_temp + 0.5*P_temp.transpose(); */
  // P_temp += MatrixXd::Identity(n, n)*0.01;

  try
  {
    P_sqrt = P_sqrt.sqrt();
  }
  catch (...)
  {
    ROS_WARN("UKF: squaring of covariance in prediction update failed.");
    throw square_root_exception();
  }

  // check whether the square root produced valid numbers
  if (!P_sqrt.array().isFinite().all())
  {
    ROS_WARN("UKF: squaring of covariance in prediction update produced NANs!!! Fix your covariances (the measurement's is probably to low..)");
    ROS_INFO_STREAM(P_debug);
    throw square_root_exception();
  }

  return P_sqrt;
}
//}

/* predict() method //{ */

template <int n_states, int n_inputs, int n_measurements>
statecov_t<n_states, n_inputs, n_measurements> UKF<n_states, n_inputs, n_measurements>::predict(const statecov_t& sc, const u_t& u, double dt, [[maybe_unused]] int param) const
{
  const auto& x = sc.state;
  const auto& P = sc.covariance;
  statecov_t ret;

  const P_t& P_sqrt = computePSqrt(P);
  const X_t S = computeSigmas(x, P_sqrt, dt);

  // propagate sigmas through the model
  X_t X;
  for (int i = 0; i < w; i++)
  {
    X.col(i) = modelIteration(S.col(i), u, dt);
  }

  // recompute the state vector
  ret.state = x_t::Zeros();
  for (int i = 0; i < w; i++)
  {
    ret.state = ret.state + m_Wm(i) * X.col(i);
  }

  // recompute the covariance
  ret.covariance = P_t::Zeros();
  for (int i = 0; i < w; i++)
  {
    ret.covariance = ret.covariance + m_Wc(i) * (X.col(i) - x) * (X.col(i) - x).transpose();
  }
  ret.covariance = ret.covariance + m_Q;

  return ret;
}

//}

/* correct() method //{ */

template <int n_states, int n_inputs, int n_measurements>
statecov_t<n_states, n_inputs, n_measurements> UKF<n_states, n_inputs, n_measurements>::correct(const statecov_t& sc, const z_t& z, const R_t& R, [[maybe_unused]] int param) const
{
  const auto& x = sc.state;
  const auto& P = sc.covariance;
  statecov_t ret;

  const P_t& P_sqrt = computePSqrt(P);
  const X_t X = computeSigmas(x, P_sqrt);

  // compute expected output sigmas
  Z_t Z_exp = m_H * X;

  // compute expected output
  z_t z_exp = z_t::Zeros();
  for (int i = 0; i < 2 * n + 1; i++)
  {
    z_exp = z_exp + m_Wm(i) * Z_exp.col(i);
  }

  // compute the expected measurement
  P_t Pyy = P_t::Zeros();
  for (int i = 0; i < 2 * n + 1; i++) {
    Pyy = Pyy + m_Wc(i) * (Z_exp.col(i) - z_exp) * (Z_exp.col(i) - z_exp).transpose();
  }
  Pyy = Pyy + R;

  // compute ..
  K_t Pxy = K_t::Zeros();
  for (int i = 0; i < 2 * n + 1; i++) {
    Pxy = Pxy + m_Wc(i) * (X.col(i) - x) * (Z_exp.col(i) - z_exp).transpose();
  }

  // compute Kalman gain
  const K_t K = Pxy * Pyy.inverse();

  // check whether the inverse produced valid numbers
  if (!K.array().isFinite().all())
  {
    ROS_ERROR("UKF: inverting of Pyy in correction update produced NANs!!! Fix your covariances (the measurement's is probably too low...)");
    throw inverse_exception();
  }

  // correct
  x = x + K * (z - z_exp);
  P = P - K * Pyy * K.transpose();

  return x;
}

//}

}  // namespace mrs_lib
