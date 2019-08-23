// clang: MatousFormat
/**  \file
     \brief Defines LKF - a class, implementing the Linear Kalman Filter \cite LKF, as well as a few specialized variants.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     \page handle LKF
 */
#ifndef LKFSYSTEMMODELS_H
#define LKFSYSTEMMODELS_H

#include <mrs_lib/kalman_filter.h>

namespace mrs_lib
{

  /* class LKF //{ */
  /**
  * \brief Implementation of the Linear Kalman filter \cite LKF.
  *
  * The Linear Kalman filter (abbreviated LKF, \cite LKF) may be used for state filtration or estimation of linear
  * stochastic discrete systems. It assumes that noise variables are sampled from multivariate gaussian distributions
  * and takes into account apriori known parameters of these distributions (namely zero means and covariance matrices,
  * which have to be specified by the user and are tunable parameters).
  *
  * The LKF C++ class itself is templated. This has its advantages and disadvantages. Main disadvantage is that it
  * may be harder to use if you're not familiar with C++ templates, which, admittedly, can get somewhat messy,
  * espetially during compilation. Another disadvantage is that if used unwisely, the compilation times can get
  * much higher when using templates. The main advantage is compile-time checking (if it compiles, then it has
  * a lower chance of crashing at runtime) and enabling more effective optimalizations during compilation. Also in case
  * of Eigen, the code is arguably more readable when you use aliases to the specific Matrix instances instead of
  * having Eigen::MatrixXd and Eigen::VectorXd everywhere.
  *
  * Example usage:
  * \include src/lkf/example.cpp
  *
  */
  template <int n_states, int n_inputs, int n_measurements>
  class LKF : public KalmanFilter<n_states, n_inputs, n_measurements>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;            /*!< \brief Length of the state vector of the system. */
    static const int m = n_inputs;            /*!< \brief Length of the input vector of the system. */
    static const int p = n_measurements;      /*!< \brief Length of the measurement vector of the system. */
    using Base_class = KalmanFilter<n, m, p>; /*!< \brief Base class of this class. */

    using x_t = typename Base_class::x_t;                /*!< \brief State vector type \f$n \times 1\f$ */
    using u_t = typename Base_class::u_t;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using z_t = typename Base_class::z_t;                /*!< \brief Measurement vector type \f$p \times 1\f$ */
    using P_t = typename Base_class::P_t;                /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
    using R_t = typename Base_class::R_t;                /*!< \brief Measurement noise covariance matrix type \f$p \times p\f$ */
    using Q_t = typename Base_class::Q_t;                /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
    using statecov_t = typename Base_class::statecov_t;  /*!< \brief Helper struct for passing around the state and its covariance in one variable */

    typedef Eigen::Matrix<double, n, n> A_t;  /*!< \brief System transition matrix type \f$n \times n\f$ */
    typedef Eigen::Matrix<double, n, m> B_t;  /*!< \brief Input to state mapping matrix type \f$n \times m\f$ */
    typedef Eigen::Matrix<double, p, n> H_t;  /*!< \brief State to measurement mapping matrix type \f$p \times n\f$ */
    typedef Eigen::Matrix<double, n, p> K_t;  /*!< \brief Kalman gain matrix type \f$n \times p\f$ */

  /*!
    * \brief This exception is thrown when taking the inverse of a matrix fails.
    *
    * You should catch this exception in your code and act accordingly if it appears
    * (e.g. reset the state and covariance or modify the measurement/process noise parameters).
    */
    struct inverse_exception : public std::exception
    {
    /*!
      * \brief Returns the error message, describing what caused the exception.
      *
      * \return  The error message, describing what caused the exception.
      */
      const char* what() const throw()
      {
        return "LKF: could not compute matrix inversion!!! Fix your covariances (the measurement's is probably too low...)";
      }
    };
    //}

  public:
  /*!
    * \brief Convenience default constructor.
    *
    * This constructor should not be used if applicable. If used, the main constructor has to be called afterwards,
    * before using this class, otherwise the LKF object is invalid (not initialized).
    */
    LKF(){};

  /*!
    * \brief The main constructor.
    *
    * \param A             The state transition matrix.
    * \param B             The input to state mapping transition matrix.
    * \param H             The state to measurement mapping transition matrix.
    */
    LKF(const A_t& A, const B_t& B, const H_t& H) : A(A), B(B), H(H){};

    /* correct() method //{ */
  /*!
    * \brief Applies the correction (update, measurement, data) step of the Kalman filter.
    *
    * This method applies the linear Kalman filter correction step to the state and covariance
    * passed in \p sc using the measurement \p z and measurement noise \p R. The parameter \p param
    * is ignored in this implementation. The updated state and covariance after the correction step
    * is returned.
    *
    * \param sc          The state and covariance to which the correction step is to be applied.
    * \param z           The measurement vector to be used for correction.
    * \param R           The measurement noise covariance matrix to be used for correction.
    * \param param       Unused parameter.
    * \return            The state and covariance after the correction update.
    */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R, [[maybe_unused]] int param = 0) const override
    {
      /* return correct_optimized(sc, z, R, H); */
      return correction_impl(sc, z, R, H);
    };
    //}

    /* predict() method //{ */
  /*!
    * \brief Applies the prediction (time) step of the Kalman filter.
    *
    * This method applies the linear Kalman filter prediction step to the state and covariance
    * passed in \p sc using the input \p u and process noise \p Q. The process noise covariance
    * \p Q is scaled by the \p dt parameter. The parameter \p param is ignored. The updated state
    * and covariance after the prediction step is returned.
    *
    * \param sc          The state and covariance to which the prediction step is to be applied.
    * \param u           The input vector to be used for prediction.
    * \param Q           The process noise covariance matrix to be used for prediction.
    * \param dt          Used to scale the process noise covariance \p Q.
    * \param param       Unused parameter.
    * \return            The state and covariance after the prediction step.
    *
    * \note Note that the \p dt parameter is only used to scale the process noise covariance \p Q it
    * does not change the system matrices #A or #B (because there is no unambiguous way to do this)!
    * If you have a changing time step duration and a dynamic system, you have to change the #A and #B
    * matrices manually.
    */
    virtual statecov_t predict(const statecov_t& sc, const u_t& u, const Q_t& Q, [[maybe_unused]] double dt, [[maybe_unused]] int param = 0) const override
    {
      statecov_t ret;
      ret.x = state_predict(A, sc.x, B, u);
      ret.P = covariance_predict(A, sc.P, Q, dt);
      return ret;
    };
    //}

  public:
    A_t A;  /*!< \brief The system transition matrix \f$n \times n\f$ */
    B_t B;  /*!< \brief The input to state mapping matrix \f$n \times m\f$ */
    H_t H;  /*!< \brief The state to measurement mapping matrix \f$p \times n\f$ */

  protected:
    /* covariance_predict() method //{ */
    static P_t covariance_predict(const A_t& A, const P_t& P, const Q_t& Q, const double dt)
    {
      return A * P * A.transpose() + dt*Q;
    }
    //}

    /* state_predict() method //{ */
    template <bool check = n_inputs>
    static inline typename std::enable_if<check == 0, x_t>::type state_predict(const A_t& A, const x_t& x, [[maybe_unused]] const B_t& B,
                                                                               [[maybe_unused]] const u_t& u)
    {
      return A * x;
    }

    template <bool check = n_inputs>
    static inline typename std::enable_if<check != 0, x_t>::type state_predict(const A_t& A, const x_t& x, const B_t& B, const u_t& u)
    {
      return A * x + B * u;
    }
    //}

  protected:
    /* invert_W() method //{ */
    static R_t invert_W(R_t W)
    {
      Eigen::ColPivHouseholderQR<R_t> qr(W);
      if (!qr.isInvertible())
      {
        // add some stuff to the tmp matrix diagonal to make it invertible
        R_t ident = R_t::Identity();
        W += 1e-9 * ident;
        qr.compute(W);
        if (!qr.isInvertible())
        {
          // never managed to make this happen except for explicitly putting NaNs in the input
          throw inverse_exception();
        }
      }
      const R_t W_inv = qr.inverse();
      return W_inv;
    }
    //}

    /* computeKalmanGain() method //{ */
    virtual K_t computeKalmanGain(const statecov_t& sc, [[maybe_unused]] const z_t& z, const R_t& R, const H_t& H) const
    {
      // calculation of the kalman gain K
      const R_t W = H * sc.P * H.transpose() + R;
      const R_t W_inv = invert_W(W);
      const K_t K = sc.P * H.transpose() * W_inv;
      return K;
    }
    //}

    /* correction_impl() method //{ */
    statecov_t correction_impl(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const
    {
      // the correction phase
      statecov_t ret;
      const K_t K = computeKalmanGain(sc, z, R, H);
      ret.x = sc.x + K * (z - (H * sc.x));
      ret.P = (P_t::Identity() - (K * H)) * sc.P;
      return ret;
    }
    //}

    // NOT USED METHODS
    /* correction_optimized() method //{ */
    // No notable performance gain was observed for the matrix sizes we use, so this is not used.
    static statecov_t correction_optimized(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H)
    {
      statecov_t ret = sc;
      const H_t B(H*sc.P.transpose());
      const K_t K((B*H.transpose() + R).ldlt().solve(B).transpose());
      ret.x.noalias() += K*(z - H*sc.x);
      ret.P.noalias() -= K*H*sc.P;
      return ret;
    }

    /* static statecov_t correction_optimized(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) */
    /* { */
    /*   statecov_t ret; */
    /*   const H_t B = H*sc.P.transpose(); */
    /*   const K_t K = (B*H.transpose() + R).partialPivLu().solve(B).transpose(); */
    /*   ret.x = sc.x + K*(z - H*sc.x); */
    /*   ret.P = sc.P - K*H*sc.P; */
    /*   return ret; */
    /* } */
    //}
  };
  //}

  /* class LKF_dt //{ */
  template <int n_states, int n_inputs, int n_measurements>
  class LKF_dt : public LKF<n_states, n_inputs, n_measurements>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;
    using Base_class = LKF<n, m, p>;

    using x_t = typename Base_class::x_t;
    using u_t = typename Base_class::u_t;
    using z_t = typename Base_class::z_t;
    using P_t = typename Base_class::P_t;
    using R_t = typename Base_class::R_t;
    using statecov_t = typename Base_class::statecov_t;
    using A_t = typename Base_class::A_t;  // measurement mapping p*n
    using B_t = typename Base_class::B_t;  // process covariance n*n
    using H_t = typename Base_class::H_t;  // measurement mapping p*n
    using Q_t = typename Base_class::Q_t;  // process covariance n*n

    using coeff_A_t = A_t;                            // matrix of constant coefficients in matrix A
    typedef Eigen::Matrix<unsigned, n, n> dtexp_A_t;  // matrix of dt exponents in matrix A
    using coeff_B_t = B_t;                            // matrix of constant coefficients in matrix B
    typedef Eigen::Matrix<unsigned, n, m> dtexp_B_t;  // matrix of dt exponents in matrix B
    //}

  public:
    LKF_dt(){};

    LKF_dt(const H_t& H, const coeff_A_t& coeff_A, const dtexp_B_t& dtexp_A, const coeff_B_t& coeff_B, const dtexp_A_t& dtexp_B)
        : coeff_A(coeff_A), dtexp_A(dtexp_A), coeff_B(coeff_B), dtexp_B(dtexp_B)
    {
      Base_class::H = H;
    };

    /* predict() method //{ */
    virtual statecov_t predict(const statecov_t& sc, const u_t& u, const Q_t& Q, double dt, [[maybe_unused]] int param = 0) const override
    {
      statecov_t ret;
      A_t A = get_A(coeff_A, dtexp_A, dt);
      B_t B = get_B(coeff_B, dtexp_B, dt);
      ret.x = Base_class::state_predict(A, sc.x, B, u);
      ret.P = Base_class::covariance_predict(A, sc.P, Q, dt);
      return ret;
    };
    //}

  public:
    coeff_A_t coeff_A;
    dtexp_A_t dtexp_A;
    coeff_B_t coeff_B;
    dtexp_B_t dtexp_B;

  public:
    /* get_A() and get_B() methods //{ */
    static A_t get_A(const coeff_A_t& coeff_A, const dtexp_A_t& dtexp_A, double dt)
    {
      A_t ret = A_t::Constant(dt);
      for (int i = 0; i < n; i++)
      {
        for (int j = 0; j < n; j++)
        {
          int exp = dtexp_A(i, j);
          for (int p = 0; p < exp; p++)
          {
            ret(i, j) *= ret(i, j);
          }
          ret(i, j) *= coeff_A(i, j);
        }
      }
      return ret;
    };
    static B_t get_B(const coeff_B_t& coeff_B, const dtexp_B_t& dtexp_B, double dt)
    {
      B_t ret = B_t::Constant(dt);
      for (int i = 0; i < n; i++)
      {
        for (int j = 0; j < n; j++)
        {
          int exp = dtexp_B(i, j);
          for (int p = 0; p < exp; p++)
          {
            ret(i, j) *= ret(i, j);
          }
          ret(i, j) *= coeff_B(i, j);
        }
      }
      return ret;
    };
    //}
  };
  //}

  /* class LKF_multiH //{ */
  template <int n_states, int n_inputs, int n_measurements>
  class LKF_multiH : public LKF_dt<n_states, n_inputs, n_measurements>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;
    using Base_class = LKF_dt<n, m, p>;

    using x_t = typename Base_class::x_t;
    using u_t = typename Base_class::u_t;
    using z_t = typename Base_class::z_t;
    using P_t = typename Base_class::P_t;
    using R_t = typename Base_class::R_t;
    using statecov_t = typename Base_class::statecov_t;
    using A_t = typename Base_class::A_t;  // measurement mapping p*n
    using B_t = typename Base_class::B_t;  // process covariance n*n
    using H_t = typename Base_class::H_t;  // measurement mapping p*n
    using Q_t = typename Base_class::Q_t;  // process covariance n*n

    using coeff_A_t = A_t;                            // matrix of constant coefficients in matrix A
    typedef Eigen::Matrix<unsigned, n, n> dtexp_A_t;  // matrix of dt exponents in matrix A
    using coeff_B_t = B_t;                            // matrix of constant coefficients in matrix B
    typedef Eigen::Matrix<unsigned, n, m> dtexp_B_t;  // matrix of dt exponents in matrix B
    //}

  public:
    LKF_multiH()
    {};

    LKF_multiH(const std::vector<H_t>& Hs, const coeff_A_t& coeff_A, const dtexp_B_t& dtexp_A, const coeff_B_t& coeff_B,
                     const dtexp_A_t& dtexp_B)
        : coeff_A(coeff_A), dtexp_A(dtexp_A), coeff_B(coeff_B), dtexp_B(dtexp_B), Hs(Hs)
    {
    };

    /* correct() method //{ */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R, int param = 0) const override
    {
      return Base_class::correction_impl(sc, z, R, Hs.at(param));
    };
    //}

  public:
    coeff_A_t coeff_A;
    dtexp_A_t dtexp_A;
    coeff_B_t coeff_B;
    dtexp_B_t dtexp_B;
    std::vector<H_t> Hs;
  };
  //}

  /* class LKF_MRS_odom //{ */
  class LKF_MRS_odom : public LKF_multiH<6, 1, 1>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = 6;
    static const int m = 1;
    static const int p = 1;
    using Base_class = LKF_multiH<n, m, p>;

    using x_t = typename Base_class::x_t;
    using u_t = typename Base_class::u_t;
    using z_t = typename Base_class::z_t;
    using P_t = typename Base_class::P_t;
    using R_t = typename Base_class::R_t;
    using statecov_t = typename Base_class::statecov_t;
    using A_t = typename Base_class::A_t;  // measurement mapping p*n
    using B_t = typename Base_class::B_t;  // process covariance n*n
    using H_t = typename Base_class::H_t;  // measurement mapping p*n
    using Q_t = typename Base_class::Q_t;  // process covariance n*n

    using coeff_A_t = A_t;                            // matrix of constant coefficients in matrix A
    typedef Eigen::Matrix<unsigned, n, n> dtexp_A_t;  // matrix of dt exponents in matrix A
    using coeff_B_t = B_t;                            // matrix of constant coefficients in matrix B
    typedef Eigen::Matrix<unsigned, n, m> dtexp_B_t;  // matrix of dt exponents in matrix B
    //}

  public:
    LKF_MRS_odom(const std::vector<H_t>& Hs, const double p1, const double p2, const double p3, const double default_dt = 1);
    virtual statecov_t predict(const statecov_t& sc, const u_t& u, const Q_t& Q, double dt, [[maybe_unused]] int param = 0) const override;
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R, int param = 0) const override;

  public:
    double p1, p2, p3;

    x_t state_predict_optimized(const x_t& x_prev, const u_t& u, double dt) const;
    P_t covariance_predict_optimized(const P_t& P, const Q_t& Q, double dt) const;
  };
  //}

}  // namespace mrs_lib

#endif // LKFSYSTEMMODELS_H
