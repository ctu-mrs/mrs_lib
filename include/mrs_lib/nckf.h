// clang: MatousFormat
/**  \file
 *   \page handle NCKF
     \brief Defines NCLKF - a class, implementing the Norm-constrained Linear Kalman Filter \cite NCKF.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#ifndef NCKFSYSTEMMODELS_H
#define NCKFSYSTEMMODELS_H

#include <mrs_lib/lkf.h>

namespace mrs_lib
{

  /* class LKF //{ */
  template <int n_states, int n_inputs, int n_measurements>
  class NCLKF : public LKF<n_states, n_inputs, n_measurements>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;
    using Base_class = LKF<n, m, p>;

    using x_t = typename Base_class::x_t;                // state vector n*1
    using u_t = typename Base_class::u_t;                // input vector m*1
    using z_t = typename Base_class::z_t;                // measurement vector p*1
    using P_t = typename Base_class::P_t;                // state covariance n*n
    using R_t = typename Base_class::R_t;                // measurement covariance p*p
    using Q_t = typename Base_class::Q_t;                // measurement covariance p*p
    using statecov_t = typename Base_class::statecov_t;  // helper struct for state and covariance

    typedef Eigen::Matrix<double, n, n> A_t;  // system matrix n*n
    typedef Eigen::Matrix<double, n, m> B_t;  // input matrix n*m
    typedef Eigen::Matrix<double, p, n> H_t;  // measurement mapping p*n
    typedef Eigen::Matrix<double, n, p> K_t;  // kalman gain n*p

    struct inverse_exception : public std::exception
    {
      const char* what() const throw()
      {
        return "NCLKF: could not compute matrix inversion!!! Fix your covariances (the measurement's is probably too low...)";
      }
    };
    //}

  public:
    NCLKF(){};

    NCLKF(const A_t& A, const B_t& B, const H_t& H, const double l)
      : l_sqrt(sqrt(l))
    {
      Base_class(A, B, H);
    };

  public:
    double l_sqrt;

  protected:
    /* calculate_kalman_gain() method //{ */
    virtual K_t calculate_kalman_gain(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const override
    {
      const R_t W = H * sc.P * H.transpose() + R;
      const R_t W_inv = Base_class::invert_W(W);
      const K_t K_orig = sc.P * H.transpose() * W_inv;

      const z_t inn = z - (H * sc.x); // innovation
      const x_t x = sc.x + K_orig * inn;
      const double inn_scale = inn.transpose() * W_inv * inn;
      
      const double x_norm = sc.x.norm();
      const K_t K = K_orig + (l_sqrt/x_norm - 1.0) * x * (inn.transpose() * W_inv) / inn_scale;
    
      return K;
    }
    //}
    
  };
  //}

}  // namespace mrs_lib

#endif // NCKFSYSTEMMODELS_H

