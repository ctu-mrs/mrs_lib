// clang: MatousFormat

#ifndef UKF_HPP
#define UKF_HPP

/**  \file
     \brief Implements UKF - a class implementing the Unscented Kalman Filter.
     \author Tomáš Báča - bacatoma@fel.cvut.cz (original implementation)
     \author Matouš Vrba - vrbamato@fel.cvut.cz (rewrite, documentation)
 */

#ifndef UKF_H
#include <mrs_lib/ukf.h>
#endif

#include <iostream>

namespace mrs_lib
{
  /* constructor //{ */

  template <int n_states, int n_inputs, int n_measurements>
  UKF<n_states, n_inputs, n_measurements>::UKF()
  {
  }

  template <int n_states, int n_inputs, int n_measurements>
  UKF<n_states, n_inputs, n_measurements>::UKF(const transition_model_t& transition_model, const observation_model_t& observation_model, const double alpha,
                                               const double kappa, const double beta)
      : m_alpha(alpha),
        m_kappa(kappa),
        m_beta(beta),
        m_Wm(W_t::Zero()),
        m_Wc(W_t::Zero()),
        m_transition_model(transition_model),
        m_observation_model(observation_model)
  {
    assert(alpha > 0.0);
    computeWeights();
  }

  //}

  /* computeWeights() //{ */

  template <int n_states, int n_inputs, int n_measurements>
  void UKF<n_states, n_inputs, n_measurements>::computeWeights()
  {
    // initialize lambda
    /* m_lambda = double(n) * (m_alpha * m_alpha - 1.0); */
    m_lambda = m_alpha * m_alpha * (double(n) + m_kappa) - double(n);

    // initialize first terms of the weights
    m_Wm(0) = m_lambda / (double(n) + m_lambda);
    m_Wc(0) = m_Wm(0) + (1.0 - m_alpha * m_alpha + m_beta);

    // initialize the rest of the weights
    for (int i = 1; i < w; i++)
    {
      m_Wm(i) = (1.0 - m_Wm(0)) / (w - 1.0);
      m_Wc(i) = m_Wm(i);
    }
  }

  //}

  /* setConstants() //{ */

  template <int n_states, int n_inputs, int n_measurements>
  // update the UKF constants
  void UKF<n_states, n_inputs, n_measurements>::setConstants(const double alpha, const double kappa, const double beta)
  {
    m_alpha = alpha;
    m_kappa = kappa;
    m_beta = beta;

    computeWeights();
  }

  //}

  /* setTransitionModel() method //{ */

  template <int n_states, int n_inputs, int n_measurements>
  void UKF<n_states, n_inputs, n_measurements>::setTransitionModel(const transition_model_t& transition_model)
  {
    m_transition_model = transition_model;
  }

  //}

  /* setObservationModel() method //{ */

  template <int n_states, int n_inputs, int n_measurements>
  void UKF<n_states, n_inputs, n_measurements>::setObservationModel(const observation_model_t& observation_model)
  {
    m_observation_model = observation_model;
  }

  //}

  /* computePaSqrt() method //{ */

  template <int n_states, int n_inputs, int n_measurements>
  typename UKF<n_states, n_inputs, n_measurements>::P_t UKF<n_states, n_inputs, n_measurements>::computePaSqrt(const P_t& P) const
  {
    // calculate the square root of the covariance matrix
    const P_t Pa = (double(n) + m_lambda) * P;

    /* Eigen::SelfAdjointEigenSolver<P_t> es(Pa); */
    Eigen::LLT<P_t> llt(Pa);
    if (llt.info() != Eigen::Success)
    {
      P_t tmp = Pa + (double(n) + m_lambda) * 1e-9 * P_t::Identity();
      llt.compute(tmp);
      if (llt.info() != Eigen::Success)
      {
        std::cout << "UKF: taking the square root of covariance during sigma point generation failed." << std::endl;
        throw square_root_exception();
      }
    }

    const P_t Pa_sqrt = llt.matrixL();
    return Pa_sqrt;
  }
  //}

  /* computeInverse() method //{ */
  template <int n_states, int n_inputs, int n_measurements>
  typename UKF<n_states, n_inputs, n_measurements>::Pzz_t UKF<n_states, n_inputs, n_measurements>::computeInverse(const Pzz_t& Pzz) const
  {
    Eigen::ColPivHouseholderQR<Pzz_t> qr(Pzz);
    if (!qr.isInvertible())
    {
      // add some stuff to the tmp matrix diagonal to make it invertible
      Pzz_t tmp = Pzz + 1e-9 * Pzz_t::Identity(Pzz.rows(), Pzz.cols());
      qr.compute(tmp);
      if (!qr.isInvertible())
      {
        // never managed to make this happen except for explicitly putting NaNs in the input
        std::cout << "UKF: could not compute matrix inversion!!! Fix your covariances (the measurement's is probably too low...)" << std::endl;
        throw inverse_exception();
      }
      std::cout << "UKF: artificially inflating matrix for inverse computation! Check your covariances (the measurement's might be too low...)" << std::endl;
    }
    Pzz_t ret = qr.inverse();
    return ret;
  }
  //}

  /* computeKalmanGain() method //{ */
  template <int n_states, int n_inputs, int n_measurements>
  typename UKF<n_states, n_inputs, n_measurements>::K_t UKF<n_states, n_inputs, n_measurements>::computeKalmanGain([[maybe_unused]] const x_t& x,
                                                                                                                   [[maybe_unused]] const z_t& inn,
                                                                                                                   const K_t& Pxz, const Pzz_t& Pzz) const
  {
    const Pzz_t Pzz_inv = computeInverse(Pzz);
    const K_t K = Pxz * Pzz_inv;
    return K;
  }
  //}

  /* computeSigmas() method //{ */
  template <int n_states, int n_inputs, int n_measurements>
  typename UKF<n_states, n_inputs, n_measurements>::X_t UKF<n_states, n_inputs, n_measurements>::computeSigmas(const x_t& x, const P_t& P) const
  {
    // calculate sigma points
    // fill in the middle of the elipsoid
    X_t S;
    S.col(0) = x;

    const P_t P_sqrt = computePaSqrt(P);
    const auto xrep = x.replicate(1, n);

    // positive sigma points
    S.template block<n, n>(0, 1) = xrep + P_sqrt;

    // negative sigma points
    S.template block<n, n>(0, n + 1) = xrep - P_sqrt;

    /* std::cout << "x: " << std::endl << x << std::endl; */
    /* std::cout << "S rowmean: " << std::endl << S.rowwise().mean() << std::endl; */

    return S;
  }
  //}

  /* predict() method //{ */

  template <int n_states, int n_inputs, int n_measurements>
  typename UKF<n_states, n_inputs, n_measurements>::statecov_t UKF<n_states, n_inputs, n_measurements>::predict(const statecov_t& sc, const u_t& u,
                                                                                                                const Q_t& Q, double dt) const
  {
    const auto& x = sc.x;
    const auto& P = sc.P;
    statecov_t ret;

    const X_t S = computeSigmas(x, P);

    // propagate sigmas through the transition model
    X_t X;
    for (int i = 0; i < w; i++)
    {
      X.col(i) = m_transition_model(S.col(i), u, dt);
    }

    /* std::cout << "x: " << std::endl << x << std::endl; */
    /* std::cout << "X rowmean: " << std::endl << X.rowwise().mean() << std::endl; */
    /* std::cout << "m_Wm sum: " << m_Wm.sum() << std::endl; */

    // recompute the state vector
    ret.x = x_t::Zero();
    for (int i = 0; i < w; i++)
    {
      // TODO: WHY DOES THIS SHIT WORK IF I SUBSTITUTE m_Wm(i) FOR 1.0/w ??
      x_t tmp = 1.0 / w * X.col(i);
      /* x_t tmp = m_Wm(i) * X.col(i); */
      ret.x += tmp;
    }

    // recompute the covariance
    ret.P = P_t::Zero();
    for (int i = 0; i < w; i++)
    {
      ret.P += m_Wc(i) * (X.col(i) - ret.x) * (X.col(i) - ret.x).transpose();
    }
    ret.P += Q;

    return ret;
  }

  //}

  /* correct() method //{ */

  template <int n_states, int n_inputs, int n_measurements>
  typename UKF<n_states, n_inputs, n_measurements>::statecov_t UKF<n_states, n_inputs, n_measurements>::correct(const statecov_t& sc, const z_t& z,
                                                                                                                const R_t& R) const
  {
    const auto& x = sc.x;
    const auto& P = sc.P;
    const X_t S = computeSigmas(x, P);

    // propagate sigmas through the observation model
    Z_t Z_exp(z.rows(), w);
    for (int i = 0; i < w; i++)
    {
      Z_exp.col(i) = m_observation_model(S.col(i));
    }

    // compute expected measurement
    z_t z_exp = z_t::Zero(z.rows());
    for (int i = 0; i < w; i++)
    {
      z_exp += m_Wm(i) * Z_exp.col(i);
    }

    // compute the covariance of measurement
    Pzz_t Pzz = Pzz_t::Zero(z.rows(), z.rows());
    for (int i = 0; i < w; i++)
    {
      Pzz += m_Wc(i) * (Z_exp.col(i) - z_exp) * (Z_exp.col(i) - z_exp).transpose();
    }
    Pzz += R;

    // compute cross covariance
    K_t Pxz = K_t::Zero(n, z.rows());
    for (int i = 0; i < w; i++)
    {
      Pxz += m_Wc(i) * (S.col(i) - x) * (Z_exp.col(i) - z_exp).transpose();
    }

    // compute Kalman gain
    const z_t inn = (z - z_exp);  // innovation
    const K_t K = computeKalmanGain(sc.x, inn, Pxz, Pzz);

    // check whether the inverse produced valid numbers
    if (!K.array().isFinite().all())
    {
      std::cout << "UKF: inverting of Pzz in correction update produced non-finite numbers!!! Fix your covariances (the measurement's is probably too low...)" << std::endl;
      throw inverse_exception();
    }

    // correct
    statecov_t ret;
    ret.x = x + K * inn;
    ret.P = P - K * Pzz * K.transpose();
    return ret;
  }

  //}

}  // namespace mrs_lib

#endif
