// clang: MatousFormat
#ifndef UKF_H
#define UKF_H

/**  \file
     \brief Defines UKF - a class implementing the Unscented Kalman Filter \cite UKF.
     \author Tomáš Báča - bacatoma@fel.cvut.cz (original implementation)
     \author Matouš Vrba - vrbamato@fel.cvut.cz (rewrite, documentation)
 */

#include <mrs_lib/kalman_filter.h>

namespace mrs_lib
{

  /**
  * \brief Implementation of the Unscented Kalman filter \cite UKF.
  *
  * The Unscented Kalman filter (abbreviated UKF, \cite UKF) is a variant of the Kalman filter, which may be used
  * for state filtration or estimation of non-linear systems as opposed to the Linear Kalman Filter
  * (which is implemented in \ref LKF). The UKF tends to be more accurate than the simpler Extended Kalman Filter,
  * espetially for highly non-linear systems. However, it is generally less stable than the LKF because of the extra
  * matrix square root in the sigma points calculation, so it is recommended to use LKF for linear systems.
  *
  * The UKF C++ class itself is templated. This has its advantages and disadvantages. Main disadvantage is that it
  * may be harder to use if you're not familiar with C++ templates, which, admittedly, can get somewhat messy,
  * espetially during compilation. Another disadvantage is that if used unwisely, the compilation times can get
  * much higher when using templates. The main advantage is compile-time checking (if it compiles, then it has
  * a lower chance of crashing at runtime) and enabling more effective optimalizations during compilation. Also in case
  * of Eigen, the code is arguably more readable when you use aliases to the specific Matrix instances instead of
  * having Eigen::MatrixXd and Eigen::VectorXd everywhere.
  *
  * \tparam n_states         number of states of the system (length of the \f$ \mathbf{x} \f$ vector).
  * \tparam n_inputs         number of inputs of the system (length of the \f$ \mathbf{u} \f$ vector).
  * \tparam n_measurements   number of measurements of the system (length of the \f$ \mathbf{z} \f$ vector).
  *
  */
  template <int n_states, int n_inputs, int n_measurements>
  class UKF : KalmanFilter<n_states, n_inputs, n_measurements>
  {
  protected:
    /* protected UKF definitions (typedefs, constants etc) //{ */
    static constexpr int n = n_states;            /*!< \brief Length of the state vector of the system. */
    static constexpr int m = n_inputs;            /*!< \brief Length of the input vector of the system. */
    static constexpr int p = n_measurements;      /*!< \brief Length of the measurement vector of the system. */
    static constexpr int w = 2 * n + 1;           /*!< \brief Number of sigma points/weights. */

    using Base_class = KalmanFilter<n, m, p>; /*!< \brief Base class of this class. */

    using X_t = typename Eigen::Matrix<double, n, w>;    /*!< \brief State sigma points matrix. */
    using Z_t = typename Eigen::Matrix<double, p, w>;    /*!< \brief Measurement sigma points matrix. */
    using Pzz_t = typename Eigen::Matrix<double, p, p>;  /*!< \brief Pzz helper matrix. */
    using K_t = typename Eigen::Matrix<double, n, p>;    /*!< \brief Kalman gain matrix. */
    //}

  public:
    /* public UKF definitions (typedefs, constants etc) //{ */
    //! state vector n*1 typedef
    using x_t = typename Base_class::x_t;
    //! input vector m*1 typedef
    using u_t = typename Base_class::u_t;
    //! measurement vector p*1 typedef
    using z_t = typename Base_class::z_t;
    //! state covariance n*n typedef
    using P_t = typename Base_class::P_t;
    //! measurement covariance p*p typedef
    using R_t = typename Base_class::R_t;
    //! process covariance n*n typedef
    using Q_t = typename Base_class::Q_t;
    //! weights vector (2n+1)*1 typedef
    using W_t = typename Eigen::Matrix<double, w, 1>;
    //! typedef of a helper struct for state and covariance
    using statecov_t = typename Base_class::statecov_t;
    //! function of the state transition model typedef
    using transition_model_t = typename std::function<x_t(const x_t&, const u_t&, double)>;
    //! function of the observation model typedef
    using observation_model_t = typename std::function<z_t(const x_t&)>;

    //! is thrown when taking the square root of a matrix fails during sigma generation
    struct square_root_exception : public std::exception
    {
      const char* what() const throw()
      {
        return "UKF: taking the square root of covariance update produced NANs!!!";
      }
    };

    //! is thrown when taking the inverse of a matrix fails during kalman gain calculation
    struct inverse_exception : public std::exception
    {
      const char* what() const throw()
      {
        return "UKF: inverting of Pzz in correction update produced NANs!!!";
      }
    };
    //}

  public:
    /* UKF constructor //{ */
  /*!
    * \brief Convenience default constructor.
    *
    * This constructor should not be used if applicable. If used, the main constructor has to be called afterwards,
    * otherwise the UKF object is invalid (not initialized).
    */
    UKF();

  /*!
    * \brief The main constructor.
    *
    * \param alpha             Scaling parameter of the sigma generation (a small positive value, e.g. 1e-3).
    * \param kappa             Secondary scaling parameter of the sigma generation (usually set to 0 or 1).
    * \param beta              Incorporates prior knowledge about the distribution (for Gaussian distribution, 2 is optimal).
    * \param transition_model  State transition model function.
    * \param observation_model Observation model function.
    */
    UKF(const transition_model_t& transition_model, const observation_model_t& observation_model, const double alpha = 1e-3, const double kappa = 1, const double beta = 2);
    //}

    /* correct() method //{ */
  /*!
    * \brief Implements the state correction step (measurement update).
    *
    * \param sc     Previous estimate of the state and covariance.
    * \param z      Measurement vector.
    * \param R      Measurement covariance matrix.
    * \returns      The state and covariance after applying the correction step.
    */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R) const override;
    //}

    /* predict() method //{ */
  /*!
    * \brief Implements the state prediction step (time update).
    *
    * \param sc     Previous estimate of the state and covariance.
    * \param u      Input vector.
    * \param Q      Process noise covariance matrix.
    * \param dt     Duration since the previous estimate.
    * \returns      The state and covariance after applying the correction step.
    */
    virtual statecov_t predict(const statecov_t& sc, const u_t& u, const Q_t& Q, double dt) const override;
    //}

    /* setConstants() method //{ */
  /*!
    * \brief Changes the Unscented Transform parameters.
    *
    * \param alpha  Scaling parameter of the sigma generation (a small positive value - e.g. 1e-3).
    * \param kappa  Secondary scaling parameter of the sigma generation (usually set to 0 or 1).
    * \param beta   Incorporates prior knowledge about the distribution (for Gaussian distribution, 2 is optimal).
    */
    void setConstants(const double alpha, const double kappa, const double beta);
    //}

    /* setTransitionModel() method //{ */
  /*!
    * \brief Changes the transition model function.
    *
    * \param transition_model   the new transition model
    */
    void setTransitionModel(const transition_model_t& transition_model);
    //}

    /* setObservationModel() method //{ */
  /*!
    * \brief Changes the observation model function.
    *
    * \param observation_model   the new observation model
    */
    void setObservationModel(const observation_model_t& observation_model);
    //}

  protected:
    /* protected methods and member variables //{ */

    void computeWeights();

    X_t computeSigmas(const x_t& x, const P_t& P) const;

    P_t computePaSqrt(const P_t& P) const;

    Pzz_t computeInverse(const Pzz_t& Pzz) const;

    virtual K_t computeKalmanGain([[maybe_unused]] const x_t& x, [[maybe_unused]] const z_t& inn, const K_t& Pxz, const Pzz_t& Pzz) const;

    double m_alpha, m_kappa, m_beta, m_lambda;
    W_t m_Wm;
    W_t m_Wc;

    transition_model_t m_transition_model;
    observation_model_t m_observation_model;

    //}
  };

}  // namespace mrs_lib


#ifndef UKF_HPP
#include <mrs_lib/impl/ukf.hpp>
#endif

#endif
