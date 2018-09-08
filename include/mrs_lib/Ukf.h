#ifndef UKF_H
#define UKF_H

/* author: Tomas Baca */

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <stdexcept>

namespace mrs_lib
{

typedef boost::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd, double)> model;

class Ukf {

public:
  Ukf(const int n, const int m, const int p, const double alpha, const double k, const double beta, const Eigen::MatrixXd R, const Eigen::MatrixXd Q,
      const Eigen::MatrixXd H, model modelIteration);

  // return estimated states
  Eigen::VectorXd getStates(void);

  // reset the filter (with new particular state vector)
  void reset(const Eigen::MatrixXd newX);

  // reset UKF with zero states
  void reset();

  // return n-th states of the estimate state vector
  double getState(const int num);

  // get the covariance matrix
  Eigen::MatrixXd getCovariance(void);

  // get element of covariance matrix
  double getCovariance(int idx);

  // set n-th states of the estimate state vector
  void setState(const int num, const double value);

  // set element of covariance matrix
  void setCovariance(const int x, const int y, const double value);

  // do iteration of the filter
  Eigen::VectorXd predictionUpdate(const double dt);

  // iterate without the correction phase
  Eigen::VectorXd correctionUpdate(const Eigen::VectorXd measurement);

  // update the UKF constants
  void setConstants(double alpha, double k, double beta);

  // set new Q
  void setQ(const Eigen::MatrixXd in);

  // set new R
  void setR(const Eigen::MatrixXd in);

  // set submatrix of covariance
  void setCovarianceSubmatrix(const Eigen::MatrixXd in, int x, int y);

  // exceptions
  struct SquareRootException : public std::exception
  {
    const char *what() const throw() {
      return "UKF: squaring of covariance in prediction update produced NANs!!!";
    }
  };

  struct InverseException : public std::exception
  {
    const char *what() const throw() {
      return "UKF: inverting of Pyy in correction update produced NANs!!!";
    }
  };

private:
  int n;  // number of states
  int m;  // number of inputs
  int p;  // number of measured states

  // UKF scaling settings
  double alpha;
  double k;
  double beta;
  double lambda;

  // UKF weights
  Eigen::VectorXd Wm, Wc;

  Eigen::MatrixXd R;  // process covariance n*n
  Eigen::MatrixXd Q;  // measurement covariance p*p
  Eigen::MatrixXd H;  // mapping states to measurement p*

  Eigen::VectorXd x;       // state vector
  Eigen::VectorXd y_exp;   // expected output (p)
  Eigen::VectorXd u;       // input vector
  Eigen::MatrixXd P;       // state vector covariance
  Eigen::MatrixXd P_sqrt;  // square root of weighted covariance
  Eigen::MatrixXd P_temp;  // temp matrix for computing square root of weighted covariance

  Eigen::MatrixXd X;      // matrix holding all sigma points (n, 2n+1)
  Eigen::MatrixXd Y_exp;  // matrix for expected sigma outpus (p, 2n+1)

  model modelIteration;  // transfere function of the system

  std::mutex ukf_mutex;

  void computeWeights(void);
};

}  // namespace mrs_lib

#endif
