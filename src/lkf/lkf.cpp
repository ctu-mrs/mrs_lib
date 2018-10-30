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
VectorXd Lkf::getStates(void) const {

  std::scoped_lock lock(lkf_mutex);

  return this->x;
}

//}

/* getCovariance() //{ */

// get the covariance matrix
MatrixXd Lkf::getCovariance(void) const {

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

MatrixXd Lkf::getA(void) const {

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

/* setP() //{ */

void Lkf::setP(const MatrixXd P) {

  std::scoped_lock lock(lkf_mutex);

  this->P = P;
}

//}

/* setR() //{ */

void Lkf::setR(const MatrixXd R) {

  std::scoped_lock lock(lkf_mutex);

  this->R = R;
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
double Lkf::getState(const int num) const {

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

/* setStates() //{ */

// set all states of the estimate state vector
void Lkf::setStates(const Eigen::VectorXd states) {

  std::scoped_lock lock(lkf_mutex);

  x = states;
}

//}

/* iterate() //{ */

// do iteration of the filter
Eigen::VectorXd Lkf::iterate(void) {

  std::scoped_lock lock(lkf_mutex);

  predictionImpl();
  correctionImpl();

  return x;
}

//}

/* iterateWithoutCorrection() //{ */

// iterate without correction
Eigen::VectorXd Lkf::iterateWithoutCorrection(void) {

  std::scoped_lock lock(lkf_mutex);

  predictionImpl();

  return x;
}

//}

/* doCorrection() //{ */

// do just the correction
Eigen::VectorXd Lkf::doCorrection(void) {

  std::scoped_lock lock(lkf_mutex);

  correctionImpl();

  return x;
}

//}

/* predictionImpl() //{ */

// implementation of the prediction step
void Lkf::predictionImpl(void) {

  // the prediction phase
  if (m > 0) {
    x = A * x + B * input;
  } else {
    x = A * x;
  }

  cov = A * cov * A.transpose() + R;

}

//}

/* correctionImpl() //{ */

// implementation of the correction step
void Lkf::correctionImpl(void) {

  // the correction phase
  MatrixXd tmp = P * cov * P.transpose() + Q;

  ColPivHouseholderQR<MatrixXd> qr(tmp);
  if (!qr.isInvertible())
  {
    // add some stuff to the tmp matrix diagonal to make it invertible
    MatrixXd ident(tmp.rows(), tmp.cols());
    ident.setIdentity();
    tmp += 1e-9*ident;
    qr.compute(tmp);
    if (!qr.isInvertible())
    {
      // never managed to make this happen except for explicitly putting NaNs in the input
      ROS_ERROR("LKF: could not compute matrix inversion!!! Fix your covariances (the measurement's is probably too low...)");
      throw InverseException();
    }
    ROS_WARN("LKF: artificially inflating matrix for inverse computation! Check your covariances (the measurement's might be too low...)");
  }
  tmp = qr.inverse();

  MatrixXd K = cov * P.transpose() * tmp;
  x          = x + K * (mes - (P * x));
  cov        = (MatrixXd::Identity(n, n) - (K * P)) * cov;

}

//}

/* /1* correctionImpl2() - for performance comparison (unused in release) //{ *1/ */

/* // implementation of the correction step */
/* void Lkf::correctionImpl2(void) { */

/*   // the correction phase */
/*   MatrixXd K = cov * P.transpose() * (P * cov * P.transpose() + Q).inverse(); */
/*   x          = x + K * (mes - (P * x)); */
/*   cov        = (MatrixXd::Identity(n, n) - (K * P)) * cov; */

/* } */

/* //} */

/* /1* correctionImpl3() - for performance comparison (unused in release) //{ *1/ */

/* // implementation of the correction step */
/* void Lkf::correctionImpl3(void) { */

/*   // the correction phase */
/*   MatrixXd to_invert = P * cov * P.transpose() + Q; */
/*   MatrixXd inverted = to_invert.inverse(); */
/*   if (!isfinite(inverted.array()).all()) */
/*   { */
/*     // add some stuff to the tmp matrix diagonal to make it invertible */
/*     MatrixXd ident(to_invert.rows(), to_invert.cols()); */
/*     ident.setIdentity(); */
/*     to_invert = P * cov * P.transpose() + Q + 1e-9*ident; */
/*     inverted = to_invert.inverse(); */
/*     if (!isfinite(inverted.array()).all()) */
/*     { */
/*       // never managed to make this happen except for explicitly putting NaNs in the input */
/*       ROS_ERROR("LKF: could not compute matrix inversion!!! Fix your covariances (the measurement's is probably too low...)"); */
/*       throw InverseException(); */
/*     } */
/*     ROS_WARN("LKF: artificially inflating matrix for inverse computation! Check your covariances (the measurement's might be too low...)"); */
/*   } */

/*   MatrixXd K = cov * P.transpose() * inverted; */
/*   x          = x + K * (mes - (P * x)); */
/*   cov        = (MatrixXd::Identity(n, n) - (K * P)) * cov; */

/* } */

/* //} */

/* /1* correctionImpl4() - for performance comparison (unused in release) //{ *1/ */

/* // implementation of the correction step */
/* void Lkf::correctionImpl4(void) { */

/*   // the correction phase */
/*   MatrixXd to_invert = P * cov * P.transpose() + Q; */
/*   MatrixXd inverted = to_invert.llt().solve(MatrixXd::Zero(to_invert.rows(), to_invert.cols())); */
/*   if (!isfinite(inverted.array()).all()) */
/*   { */
/*     TODO */
/*     // add some stuff to the tmp matrix diagonal to make it invertible */
/*     MatrixXd ident(to_invert.rows(), to_invert.cols()); */
/*     ident.setIdentity(); */
/*     to_invert = P * cov * P.transpose() + Q + 1e-9*ident; */
/*     inverted = to_invert.inverse(); */
/*     if (!isfinite(inverted.array()).all()) */
/*     { */
/*       // never managed to make this happen except for explicitly putting NaNs in the input */
/*       ROS_ERROR("LKF: could not compute matrix inversion!!! Fix your covariances (the measurement's is probably too low...)"); */
/*       throw InverseException(); */
/*     } */
/*     ROS_WARN("LKF: artificially inflating matrix for inverse computation! Check your covariances (the measurement's might be too low...)"); */
/*   } */

/*   MatrixXd K = cov * P.transpose() * inverted; */
/*   x          = x + K * (mes - (P * x)); */
/*   cov        = (MatrixXd::Identity(n, n) - (K * P)) * cov; */

/* } */

/* //} */

}  // namespace mrs_lib
