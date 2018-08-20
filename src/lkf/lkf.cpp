/* author: Tomas Baca */

#include <ros/ros.h>
#include <mrs_lib/Lkf.h>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

namespace mrs_lib
{

Lkf::Lkf(const int n, const int m, const int p, const MatrixXd &A, const MatrixXd &B, const MatrixXd &R, const MatrixXd &Q, const MatrixXd &P) {

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

// get the state vector
VectorXd Lkf::getStates(void) {

  VectorXd temp = VectorXd::Zero(n);

  lkf_mutex.lock();
  { temp = this->x; }
  lkf_mutex.unlock();

  return temp;
}

// get the covariance matrix
MatrixXd Lkf::getCovariance(void) {

  MatrixXd temp = MatrixXd::Zero(n, n);

  lkf_mutex.lock();
  { temp = this->cov; }
  lkf_mutex.unlock();

  return temp;
}

void Lkf::setCovariance(const MatrixXd &cov) {

  lkf_mutex.lock();
  { this->cov = cov; }
  lkf_mutex.unlock();
}

MatrixXd Lkf::getA(void) {

  MatrixXd temp = MatrixXd::Zero(n, n);

  lkf_mutex.lock();
  { temp = this->A; }
  lkf_mutex.unlock();

  return temp;
}

void Lkf::setA(const MatrixXd &A) {

  lkf_mutex.lock();
  { this->A = A; }
  lkf_mutex.unlock();
}

void Lkf::setB(const MatrixXd &B) {

  lkf_mutex.lock();
  { this->B = B; }
  lkf_mutex.unlock();
}

void Lkf::setP(const MatrixXd &P) {

  lkf_mutex.lock();
  { this->P = P; }
  lkf_mutex.unlock();
}

// reset the filter with particular new states
void Lkf::reset(const MatrixXd &newX) {

  lkf_mutex.lock();
  {
    x   = newX;
    cov = MatrixXd::Identity(n, n);
  }
  lkf_mutex.unlock();
}

// set new measurement and its covariance
void Lkf::setMeasurement(const Eigen::VectorXd &newMes, const Eigen::MatrixXd &newCov) {

  lkf_mutex.lock();
  {
    mes = newMes;
    Q   = newCov;
  }
  lkf_mutex.unlock();
}

// set new measurement
void Lkf::setMeasurement(const Eigen::VectorXd &newMes) {

  lkf_mutex.lock();
  { mes = newMes; }
  lkf_mutex.unlock();
}

// set new input vector
void Lkf::setInput(const Eigen::VectorXd &newInput) {

  lkf_mutex.lock();
  { input = newInput; }
  lkf_mutex.unlock();
}

// return n-th states of the estimate state vector
double Lkf::getState(const int num) {

  double temp;

  lkf_mutex.lock();
  { temp = x[num]; }
  lkf_mutex.unlock();

  return temp;
}

// set n-th states of the estimate state vector
void Lkf::setState(const int num, const double value) {

  lkf_mutex.lock();
  { x[num] = value; }
  lkf_mutex.unlock();
}

// do iteration of the filter
Eigen::VectorXd Lkf::iterate(void) {

  VectorXd temp = VectorXd::Zero(n);

  lkf_mutex.lock();
  {
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

    temp = x;
  }
  lkf_mutex.unlock();

  return temp;
}

// iterate without correction
Eigen::VectorXd Lkf::iterateWithoutCorrection(void) {

  VectorXd temp = VectorXd::Zero(n);

  lkf_mutex.lock();
  {
    // the prediction phase
    if (m > 0) {
      x = A * x + B * input;
    } else {
      x = A * x;
    }

    cov = A * cov * A.transpose() + R;

    temp = x;
  }
  lkf_mutex.unlock();

  return temp;
}

// do just the correction
Eigen::VectorXd Lkf::doCorrection(void) {

  VectorXd temp = VectorXd::Zero(n);

  lkf_mutex.lock();
  {
    // the correction phase
    MatrixXd K = cov * P.transpose() * (P * cov * P.transpose() + Q).inverse();
    x          = x + K * (mes - (P * x));
    cov        = (MatrixXd::Identity(n, n) - (K * P)) * cov;

    temp = x;
  }
  lkf_mutex.unlock();

  return temp;
}
}  // namespace mrs_lib
