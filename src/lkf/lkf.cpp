/* author: Tomas Baca */

#include <ros/ros.h>
#include <mrs_lib/Lkf.h>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

namespace mrs_lib
{

/* constructor //{ */

Lkf::Lkf(const int n, const int m, const int p, const MatrixXd A, const MatrixXd B, const MatrixXd R, const MatrixXd Q, const MatrixXd P) {

  this->n = n;
  this->m = m;
  this->p = p;

  this->A = A;
  this->B = B;
  this->R = R;
  this->Q = Q;
  this->P = P;

  x     = VectorXd::Zero(n);
  cov   = MatrixXd::Identity(n, n);
  input = VectorXd::Zero(m);
  mes   = VectorXd::Zero(p);
}

//}

/* getStates() //{ */

// get the state vector
VectorXd Lkf::getStates(void) {

  std::scoped_lock lock(lkf_mutex);

  return this->x;
}

//}

/* getCovariance() //{ */

// get the covariance matrix
MatrixXd Lkf::getCovariance(void) {

  std::scoped_lock lock(lkf_mutex);

  return this->cov;
}

//}

/* setCovariance() //{ */

void Lkf::setCovariance(const MatrixXd cov) {

  std::scoped_lock lock(lkf_mutex);

  this->cov = cov;
}

//}

/* getA() //{ */

MatrixXd Lkf::getA(void) {

  std::scoped_lock lock(lkf_mutex);

  return this->A;
}

//}

/* setA() //{ */

void Lkf::setA(const MatrixXd A) {

  std::scoped_lock lock(lkf_mutex);

  this->A = A;
}

//}

/* setB() //{ */

void Lkf::setB(const MatrixXd B) {

  std::scoped_lock lock(lkf_mutex);

  this->B = B;
}

//}

/* setP //{ */

void Lkf::setP(const MatrixXd P) {

  std::scoped_lock lock(lkf_mutex);

  this->P = P;
}

//}

/* reset() //{ */

// reset the filter with particular new states
void Lkf::reset(const MatrixXd newX) {

  std::scoped_lock lock(lkf_mutex);

  x   = newX;
  cov = MatrixXd::Identity(n, n);
}

//}

/* setMeaurement() //{ */

// set new measurement and its covariance
void Lkf::setMeasurement(const Eigen::VectorXd newMes, const Eigen::MatrixXd newCov) {

  std::scoped_lock lock(lkf_mutex);

  mes = newMes;
  Q   = newCov;
}

// set new measurement
void Lkf::setMeasurement(const Eigen::VectorXd newMes) {

  std::scoped_lock lock(lkf_mutex);

  mes = newMes;
}

//}

/* setInput() //{ */

// set new input vector
void Lkf::setInput(const Eigen::VectorXd newInput) {

  std::scoped_lock lock(lkf_mutex);

  input = newInput;
}

//}

/* getState() //{ */

// return n-th states of the estimate state vector
double Lkf::getState(const int num) {

  std::scoped_lock lock(lkf_mutex);

  return x[num];
}

//}

/* setState() //{ */

// set n-th states of the estimate state vector
void Lkf::setState(const int num, const double value) {

  std::scoped_lock lock(lkf_mutex);

  x[num] = value;
}

//}

/* iterate() //{ */

// do iteration of the filter
Eigen::VectorXd Lkf::iterate(void) {

  std::scoped_lock lock(lkf_mutex);

  // the prediction phase
  if (m > 0) {
    x = A * x + B * input;
  } else {
    x = A * x;
  }

  cov = A * cov * A.transpose() + R;

  // the correction phase
  MatrixXd K = cov * P.transpose() * (P * cov * P.transpose() + Q).inverse();
  x          = x + K * (mes - (P * x));
  cov        = (MatrixXd::Identity(n, n) - (K * P)) * cov;

  return x;
}

//}

/* iterateWithoutCorrection() //{ */

// iterate without correction
Eigen::VectorXd Lkf::iterateWithoutCorrection(void) {

  std::scoped_lock lock(lkf_mutex);

  // the prediction phase
  if (m > 0) {
    x = A * x + B * input;
  } else {
    x = A * x;
  }

  cov = A * cov * A.transpose() + R;

  return x;
}

//}

/* doCorrection() //{ */

// do just the correction
Eigen::VectorXd Lkf::doCorrection(void) {

  std::scoped_lock lock(lkf_mutex);

  // the correction phase
  MatrixXd K = cov * P.transpose() * (P * cov * P.transpose() + Q).inverse();
  x          = x + K * (mes - (P * x));
  cov        = (MatrixXd::Identity(n, n) - (K * P)) * cov;

  return x;
}

//}
}  // namespace mrs_lib
