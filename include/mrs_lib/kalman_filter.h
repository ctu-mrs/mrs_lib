#ifndef SYSTEMMODEL_H
#define SYSTEMMODEL_H

#include <Eigen/Dense>

namespace mrs_lib
{
  /* KalmanFilter virtual class //{ */
  template <int n_states, int n_inputs, int n_measurements>
  class KalmanFilter
  {
    public:
    /* states, inputs etc. definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;

    typedef Eigen::Matrix<double, n, 1> x_t;  // state vector n*1
    typedef Eigen::Matrix<double, m, 1> u_t;  // input vector m*1
    typedef Eigen::Matrix<double, p, 1> z_t;  // measurement vector p*1

    typedef Eigen::Matrix<double, n, n> P_t;  // state covariance n*n
    typedef Eigen::Matrix<double, p, p> R_t;  // measurement covariance p*p
    typedef Eigen::Matrix<double, n, n> Q_t;  // process noise covariance n*n
    //}

    /* statecov_t struct //{ */
    struct statecov_t
    {
      x_t x;
      P_t P;
    };
    //}

    public:
      virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R, int param) const = 0;
      virtual statecov_t predict(const statecov_t& sc, const u_t& u, const Q_t& Q, double dt, int param) const = 0;
  };
  //}
}

#endif // SYSTEMMODEL_H
