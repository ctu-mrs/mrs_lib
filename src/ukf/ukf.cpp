/* author: Tomas Baca */

#include <ros/ros.h>
#include <mrs_lib/Ukf.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

namespace mrs_lib
{

/* constructor //{ */

Ukf::Ukf(const int n, const int m, const int p, const double alpha, const double k, const double beta, const Eigen::MatrixXd R, const Eigen::MatrixXd Q,
         const Eigen::MatrixXd H, model modelIteration) {

  this->n = n;
  this->m = m;
  this->p = p;

  this->alpha = alpha;
  this->k     = k;
  this->beta  = beta;

  // initialize weights
  Wm = VectorXd::Zero(2 * n + 1);
  Wc = VectorXd::Zero(2 * n + 1);

  computeWeights();

  this->R = R;
  this->Q = Q;
  this->H = H;

  x      = VectorXd::Zero(n);
  y_exp  = VectorXd::Zero(p);
  P      = MatrixXd::Identity(n, n) * 0.1;  // covariance
  P_sqrt = MatrixXd::Zero(n, n);
  P_temp = MatrixXd::Zero(n, n);
  X      = MatrixXd::Zero(n, 2 * n + 1);
  Y_exp  = MatrixXd::Zero(p, 2 * n + 1);

  this->modelIteration = modelIteration;
}

//}

/* computeWeights() //{ */

void Ukf::computeWeights(void) {

  // initialize lambda
  lambda = double(n) * (alpha * alpha - double(1));

  // initialize first terms of the weights
  Wm(0) = lambda / (double(n) + lambda);
  Wc(0) = Wm(0) + (double(1) - alpha * alpha + beta);

  // initialize the rest of the weights
  for (int i = 1; i < 2 * n + 1; i++) {
    Wm(i) = double(1) / (double(2) * (double(n) + lambda));
    Wc(i) = Wm(i);
  }
}

//}

/* setQ() //{ */

// set the measurement covariance
void Ukf::setQ(const Eigen::MatrixXd in) {

  std::scoped_lock lock(ukf_mutex);

  this->Q = in;
}

//}

/* setR() //{ */

// set the process covariance
void Ukf::setR(const Eigen::MatrixXd in) {

  std::scoped_lock lock(ukf_mutex);

  this->R = in;
}

//}

/* setCovarianceSubmatrix() //{ */

// set subblock of covariance matrix
// x, y are coordinates of the left-up corner of the submatrix
void Ukf::setCovarianceSubmatrix(const Eigen::MatrixXd in, int x, int y) {

  std::scoped_lock lock(ukf_mutex);

  // check the dimensions
  if ((x + in.cols()) <= n) {

    P.block(x, y, in.cols(), in.rows()) = in;

  } else {

    ROS_ERROR_THROTTLE(1, "UKF error, setCovarianceSubmatrix out of bounds!");
  }
}

//}

/* setConstants() //{ */

// update the UKF constants
void Ukf::setConstants(double alpha, double k, double beta) {

  std::scoped_lock lock(ukf_mutex);

  this->alpha = alpha;
  this->k     = k;
  this->beta  = beta;

  computeWeights();
}

//}

/* getStates() //{ */

// get the state vector
VectorXd Ukf::getStates(void) {

  std::scoped_lock lock(ukf_mutex);

  return this->x;
}

//}

/* getCovariance() //{ */

// get the covariance matrix
MatrixXd Ukf::getCovariance(void) {

  std::scoped_lock lock(ukf_mutex);

  return this->P;
}

// get element of covariance matrix
double Ukf::getCovariance(int idx) {

  std::scoped_lock lock(ukf_mutex);

  return P(idx);
}

//}

/* reset() //{ */

// reset the filter with particular new states
void Ukf::reset(const MatrixXd newX) {

  std::scoped_lock lock(ukf_mutex);

  x = newX;
  P = MatrixXd::Identity(n, n);
}

// reset the filter with zero states
void Ukf::reset() {

  std::scoped_lock lock(ukf_mutex);

  x = VectorXd::Zero(n);
  P = MatrixXd::Identity(n, n);
}

//}

/* getState() //{ */

// return n-th states of the estimate state vector
double Ukf::getState(const int num) {

  std::scoped_lock lock(ukf_mutex);

  return x(num);
}

// set n-th states of the estimate state vector
void Ukf::setState(const int num, const double value) {

  std::scoped_lock lock(ukf_mutex);

  x[num] = value;
}

//}

/* setCovariance() //{ */

// set element of covariance matrix
void Ukf::setCovariance(const int x, const int y, const double value) {

  std::scoped_lock lock(ukf_mutex);

  P(x, y) = value;
}

//}

/* setCovariance() //{ */

// set element of covariance matrix
void Ukf::setCovariance(const Eigen::MatrixXd in) {

  std::scoped_lock lock(ukf_mutex);

  P = in;
}

//}

/* predictionUpdate() //{ */

// do iteration of the filter
Eigen::VectorXd Ukf::predictionUpdate(const double dt) {

  std::scoped_lock lock(ukf_mutex);

  // calculate the square root of the covariance matrix
  P_temp = (double(n) + lambda) * P;

  Eigen::MatrixXd pes = MatrixXd::Zero(n, n);
  pes                 = P_temp;

  // reguralization of P_temp

  EigenSolver<MatrixXd> es(P_temp);

  for (int i = 0; i < n; ++i) {
    if (es.eigenvalues().col(0)[i].real() <= 0) {

      ROS_ERROR("UKF: Eigen values of P_temp are <= 0!");
      throw SquareRootException();
    }
  }

  // if there are problems with square rooting P, try this:
  /* P_temp = 0.5*P_temp + 0.5*P_temp.transpose(); */
  // P_temp += MatrixXd::Identity(n, n)*0.01;

  try {
    P_temp = P_temp.sqrt();
  }
  catch (...) {

    ROS_WARN("UKF: squaring of covariance in prediction update failed.");
    throw SquareRootException();
  }

  // check whether the square root produced valid numbers
  if (!isfinite(P_temp.array()).all()) {

    ROS_WARN("UKF: squaring of covariance in prediction update produced NANs!!! Fix your covariances (the measurement's is probably to low..)");
    ROS_INFO_STREAM(pes);
    throw SquareRootException();
  }

  // if now exception is thrown, copy the temp value
  P_sqrt = P_temp;

  // calculate sigma points
  // fill in the middle of the elipsoid
  X.col(0) = x;

  // positive sigma points
  for (int i = 1; i <= n; i++) {
    X.col(i) = x + P_sqrt.row(i - 1).transpose();
  }

  // negative sigma points
  for (int i = n + 1; i <= 2 * n; i++) {
    X.col(i) = x - P_sqrt.row(i - n - 1).transpose();
  }

  // update sigmas by the model
  for (int i = 0; i < 2 * n + 1; i++) {
    X.col(i) = modelIteration(X.col(i), VectorXd::Zero(1), dt);
  }

  // recompute the state vector
  x = VectorXd::Zero(n);
  for (int i = 0; i < 2 * n + 1; i++) {
    x = x + Wm(i) * X.col(i);
  }

  // recompute the covariance
  P = MatrixXd::Zero(n, n);
  for (int i = 0; i < 2 * n + 1; i++) {
    P = P + Wc(i) * (X.col(i) - x) * (X.col(i) - x).transpose();
  }
  P = P + R;

  // compute expected output sigmas
  Y_exp = H * X;

  // compute expected output
  y_exp = VectorXd::Zero(p);
  for (int i = 0; i < 2 * n + 1; i++) {
    y_exp = y_exp + Wm(i) * Y_exp.col(i);
  }

  return x;
}

//}

/* correctionUpdate() //{ */

// iterate without correction
Eigen::VectorXd Ukf::correctionUpdate(const Eigen::VectorXd measurement) {

  std::scoped_lock lock(ukf_mutex);

  // compute the expected measurement
  MatrixXd Pyy = MatrixXd::Zero(p, p);
  for (int i = 0; i < 2 * n + 1; i++) {
    Pyy = Pyy + Wc(i) * (Y_exp.col(i) - y_exp) * (Y_exp.col(i) - y_exp).transpose();
  }
  Pyy = Pyy + Q;

  // compute ..
  MatrixXd Pxy = MatrixXd::Zero(n, p);
  for (int i = 0; i < 2 * n + 1; i++) {
    Pxy = Pxy + Wc(i) * (X.col(i) - x) * (Y_exp.col(i) - y_exp).transpose();
  }

  // compute Kalman gain
  MatrixXd K = MatrixXd::Zero(n, p);
  K          = Pxy * Pyy.inverse();

  // check whether the inverse produced valid numbers
  if (!isfinite(K.array()).all()) {

    ROS_ERROR("UKF: inverting of Pyy in correction update produced NANs!!! Fix your covariances (the measurement's is probably too low...)");
    throw InverseException();
  }

  // correct
  x = x + K * (measurement - y_exp);
  P = P - K * Pyy * K.transpose();

  return x;
}

//}

}  // namespace mrs_lib
