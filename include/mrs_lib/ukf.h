// clang: MatousFormat
#ifndef UKF_H
#define UKF_H

/**  \file
     \brief Defines UKF - a class implementing the Unscented Kalman Filter.
     \author Tomáš Báča - bacatoma@fel.cvut.cz (original implementation)
     \author Matouš Vrba - vrbamato@fel.cvut.cz (rewrite, documentation)
 */

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <stdexcept>
#include <mrs_lib/system_model.h>

namespace mrs_lib
{

  template <int n_states, int n_inputs, int n_measurements>
  class UKF : SystemModel<n_states, n_inputs, n_measurements>
  {
  private:
    /* private UKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;
    static const int w = 2*n+1; // number of sigma points/weights

    using Base_class = SystemModel<n, m, p>;

    using X_t = typename Eigen::Matrix<double, n, w>;  // state sigma points matrix n*w
    using Z_t = typename Eigen::Matrix<double, p, w>;  // measurement sigma points matrix p*w
    //}

  public:
    /* public UKF definitions (typedefs, constants etc) //{ */
    using x_t = typename Base_class::x_t;                // state vector n*1
    using u_t = typename Base_class::u_t;                // input vector m*1
    using z_t = typename Base_class::z_t;                // measurement vector p*1
    using P_t = typename Base_class::P_t;                // state covariance n*n
    using R_t = typename Base_class::R_t;                // measurement covariance p*p
    using statecov_t = typename Base_class::statecov_t;  // helper struct for state and covariance

    using H_t = typename Eigen::Matrix<double, p, n>;      // measurement mapping p*n
    using Q_t = typename Eigen::Matrix<double, n, n>;      // process covariance n*n
    using K_t = typename Eigen::Matrix<double, n, p>;      // kalman gain n*p
    using W_t = typename Eigen::Matrix<double, w, 1>;  // weights vector

    using model_t = typename std::function<x_t(const x_t&, const u_t&, double)>;

    // exceptions
    struct square_root_exception : public std::exception
    {
      const char* what() const throw()
      {
        return "UKF: taking the square root of covariance in prediction update produced NANs!!!";
      }
    };

    struct inverse_exception : public std::exception
    {
      const char* what() const throw()
      {
        return "UKF: inverting of Pyy in correction update produced NANs!!!";
      }
    };
    //}

  public:
    /* UKF constructor //{ */
    UKF();
    UKF(const double alpha, const double k, const double beta, const R_t& R, const Q_t& Q, const H_t& H, const model_t& model);
    //}

    /* correct() method //{ */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R, [[maybe_unused]] int param = 0) const override;
    //}

    /* predict() method //{ */
    virtual statecov_t predict(const statecov_t& sc, const u_t& u, double dt, [[maybe_unused]] int param = 0) const override;
    //}

    /* setConstants() method //{ */
    void setConstants(const double alpha, const double k, const double beta);
    //}

  private:
    /* private methods and member variables //{ */
    
    void computeWeights();

    X_t computeSigmas(const x_t& x, const P_t& P_sqrt) const;

    P_t computePSqrt(const P_t& P) const;
    
    double m_alpha, m_k, m_beta, m_lambda;
    W_t m_Wm;
    W_t m_Wc;
    
    R_t m_R;
    Q_t m_Q;
    H_t m_H;
    
    model_t m_model;
    
    //}

  };

}  // namespace mrs_lib

#endif