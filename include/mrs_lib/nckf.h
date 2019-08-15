// clang: MatousFormat
/**  \file
 *   \page handle NCKF
     \brief Defines NCLKF - a class, implementing the Norm-constrained Linear Kalman Filter \cite NCKF.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#ifndef NCKFSYSTEMMODELS_H
#define NCKFSYSTEMMODELS_H

#include <mrs_lib/lkf.h>
#include <mrs_lib/ukf.h>

namespace mrs_lib
{

  /* class NCLKF //{ */
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

    using A_t = typename Base_class::A_t;  // system matrix n*n
    using B_t = typename Base_class::B_t;  // input matrix n*m
    using H_t = typename Base_class::H_t;  // measurement mapping p*n
    using K_t = typename Base_class::K_t;  // kalman gain n*p
    //}

  public:
    NCLKF(){};

    NCLKF(const A_t& A, const B_t& B, const H_t& H, const double l) : Base_class(A, B, H), l_sqrt(sqrt(l)) {};

  public:
    double l_sqrt;

  protected:
    /* computeKalmanGain() method //{ */
    virtual K_t computeKalmanGain(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const override
    {
      const R_t W = H * sc.P * H.transpose() + R;
      const R_t W_inv = Base_class::invert_W(W);
      const K_t K_orig = sc.P * H.transpose() * W_inv;

      const z_t inn = z - (H * sc.x); // innovation
      const x_t x = sc.x + K_orig * inn;
      const double inn_scale = inn.transpose() * W_inv * inn;
      
      const double x_norm = x.norm();
      const K_t K = K_orig + (l_sqrt/x_norm - 1.0) * x * (inn.transpose() * W_inv) / inn_scale;
    
      return K;
    }
    //}
    
  };
  //}

  /* class NCUKF //{ */
  template <int n_states, int n_inputs, int n_measurements>
  class NCUKF : public UKF<n_states, n_inputs, n_measurements>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;
    using Base_class = UKF<n, m, p>;

    using x_t = typename Base_class::x_t;                // state vector n*1
    using u_t = typename Base_class::u_t;                // input vector m*1
    using z_t = typename Base_class::z_t;                // measurement vector p*1
    using P_t = typename Base_class::P_t;                // state covariance n*n
    using R_t = typename Base_class::R_t;                // measurement covariance p*p
    using Q_t = typename Base_class::Q_t;                // measurement covariance p*p
    using statecov_t = typename Base_class::statecov_t;  // helper struct for state and covariance

    using transition_model_t = typename Base_class::transition_model_t;
    using observation_model_t = typename Base_class::observation_model_t;

    using X_t = typename Base_class::X_t;  // state sigma points matrix n*w
    using Z_t = typename Base_class::Z_t;  // measurement sigma points matrix p*w
    using Pzz_t = typename Base_class::Pzz_t;  // Pzz helper matrix p*n
    using K_t = typename Base_class::K_t;  // kalman gain n*p
    //}

  public:
    NCUKF(){};

    NCUKF(const transition_model_t& transition_model, const observation_model_t& observation_model, const double l, const double alpha = 1e-3, const double kappa = 1, const double beta = 2)
     : Base_class(transition_model, observation_model, alpha, kappa, beta), l_sqrt(sqrt(l)) {};

  public:
    /* correct() method //{ */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R, [[maybe_unused]] int param = 0) const override
    {
      const auto& x = sc.x;
      const auto& P = sc.P;
      const X_t S = Base_class::computeSigmas(x, P);
    
      // propagate sigmas through the observation model
      Z_t Z_exp;
      for (int i = 0; i < Base_class::w; i++)
      {
        Z_exp.col(i) = Base_class::m_observation_model(S.col(i));
      }
    
      // compute expected measurement
      z_t z_exp = z_t::Zero();
      for (int i = 0; i < Base_class::w; i++)
      {
        z_exp += Base_class::m_Wm(i) * Z_exp.col(i);
      }
    
      // compute the covariance of measurement
      Pzz_t Pzz = Pzz_t::Zero();
      for (int i = 0; i < Base_class::w; i++)
      {
        Pzz += Base_class::m_Wc(i) * (Z_exp.col(i) - z_exp) * (Z_exp.col(i) - z_exp).transpose();
      }
      Pzz += R;
    
      // compute cross covariance
      K_t Pxz = K_t::Zero();
      for (int i = 0; i < Base_class::w; i++)
      {
        Pxz += Base_class::m_Wc(i) * (S.col(i) - x) * (Z_exp.col(i) - z_exp).transpose();
      }
    
      // compute Kalman gain
      const z_t inn = (z - z_exp); // innovation
      const K_t K = computeKalmanGain(sc.x, inn, Pxz, Pzz);
    
      // check whether the inverse produced valid numbers
      if (!K.array().isFinite().all())
      {
        ROS_ERROR("UKF: inverting of Pzz in correction update produced non-finite numbers!!! Fix your covariances (the measurement's is probably too low...)");
        throw typename Base_class::inverse_exception();
      }
    
      // correct
      statecov_t ret;
      ret.x = x + K * inn;
      ret.P = P - K * Pxz.transpose() - Pxz * K.transpose() + K * Pzz * K.transpose();
      return ret;
    }
    //}

  protected:
    double l_sqrt;

  protected:
    /* computeKalmanGain() method //{ */
    virtual K_t computeKalmanGain(const x_t& x, const z_t& inn, const K_t& Pxz, const Pzz_t& Pzz) const override
    {
      const Pzz_t Pzz_inv = Base_class::computeInverse(Pzz);
      const K_t K_orig = Pxz * Pzz_inv;

      const x_t x_pred = x + K_orig * inn;
      const double inn_scale = inn.transpose() * Pzz_inv * inn;
      
      const double x_norm = x_pred.norm();
      const K_t K = K_orig + (l_sqrt/x_norm - 1.0) * x_pred * (inn.transpose() * Pzz_inv) / inn_scale;
    
      return K;
    }
    //}

  };
  //}

}  // namespace mrs_lib

#endif // NCKFSYSTEMMODELS_H

