#ifndef LKF_H
#define LKF_H

/* author: Tomas Baca */

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <mutex>

namespace mrs_lib
{

class Lkf {

public:
  Lkf(const int n, const int m, const int p, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& P);

  Lkf(const Lkf& lkf);

  // return estimated states
  Eigen::VectorXd getStates(void) const;

  // reset the filter (with new particular state vector)
  void reset(const Eigen::MatrixXd newX);

  // set new measurement and its covariance
  void setMeasurement(const Eigen::VectorXd newMes, const Eigen::MatrixXd newCov);

  // set new measurement
  void setMeasurement(const Eigen::VectorXd newMes);

  // set new input vector
  void setInput(const Eigen::VectorXd newInput);

  // return n-th states of the estimate state vector
  double getState(const int num) const;

  // get the covariance matrix
  Eigen::MatrixXd getCovariance(void) const;

  // get main matrix
  Eigen::MatrixXd getA(void) const;

  // set main matrix
  void setA(const Eigen::MatrixXd A);

  // set input matrix
  void setB(const Eigen::MatrixXd B);

  // set measurement mapping matrix
  void setP(const Eigen::MatrixXd P);

  // set process noise
  void setR(const Eigen::MatrixXd R);

  // set covariance
  void setCovariance(const Eigen::MatrixXd cov);

  // set n-th states of the estimate state vector
  void setState(const int num, const double value);

  // set all states of the estimate state vector
  void setStates(const Eigen::VectorXd states);

  // do iteration of the filter
  Eigen::VectorXd iterate(void);

  // iterate without the correction phase
  Eigen::VectorXd iterateWithoutCorrection(void);

  // do just the correction
  virtual Eigen::VectorXd doCorrection(void);

  struct InverseException : public std::exception
  {
    const char *what() const throw() {
      return "LKF: could not compute matrix inversion in correction update!!!";
    }
  };

  /* /1* methods for performance comparison (unused in release) //{ *1/ */
  /* void correctionImpl(); */
  /* void correctionImpl2(); */
  /* void correctionImpl3(); */
  /* void correctionImpl4(); */
  /* //} */

private:
  int n;  // number of states
  int m;  // number of inputs
  int p;  // number of measured states

  Eigen::MatrixXd A;  // system matrix n*n
  Eigen::MatrixXd B;  // input matrix n*m
  Eigen::MatrixXd R;  // process covariance n*n
  Eigen::MatrixXd Q;  // measurement covariance p*p
  Eigen::MatrixXd P;  // measurement mapping p*n

  Eigen::VectorXd x;    // state vector
  Eigen::MatrixXd cov;  // state vector covariance

  Eigen::VectorXd mes;            // the last measurement
  Eigen::MatrixXd mesCovariance;  // last measurement covariance

  Eigen::VectorXd input;  // system input vector

  mutable std::mutex lkf_mutex;

  void predictionImpl();
  void correctionImpl();
};

}  // namespace mrs_lib

#endif
