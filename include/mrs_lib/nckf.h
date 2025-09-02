// clang: MatousFormat
/**  \file NCKF
     \brief Defines NCLKF - a class, implementing the Norm-constrained Linear Kalman Filter \cite NCLKF and NCUKF for the Norm-constrained Unscented Kalman
   Filter \cite NCUKF. \author Matouš Vrba - vrbamato@fel.cvut.cz
 */
#ifndef NCKFSYSTEMMODELS_H
#define NCKFSYSTEMMODELS_H

#include <mrs_lib/lkf.h>
#include <mrs_lib/ukf.h>

namespace mrs_lib
{

  /* class NCLKF //{ */
  /**
   * \brief This class implements the norm-constrained linear Kalman filter \cite NCLKF.
   *
   * Note that the class is templated. The template parameters specify the number of states,
   * number of inputs and number of measurements of the system. The last template parameter
   * specifies the number of states which are norm-constrained.
   *
   * The norm constraint is specified in the constructor and applied to the whole state
   * vector of the system.
   *
   */
  template <int n_states, int n_inputs, int n_measurements>
  class NCLKF : public LKF<n_states, n_inputs, n_measurements>
  {
  public:
    /* NCLKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;       /*!< \brief Length of the state vector of the system. */
    static const int m = n_inputs;       /*!< \brief Length of the input vector of the system. */
    static const int p = n_measurements; /*!< \brief Length of the measurement vector of the system. */
    using Base_class = LKF<n, m, p>;     /*!< \brief Base class of this class. */

    using x_t = typename Base_class::x_t;               /*!< \brief State vector type \f$n \times 1\f$ */
    using u_t = typename Base_class::u_t;               /*!< \brief Input vector type \f$m \times 1\f$ */
    using z_t = typename Base_class::z_t;               /*!< \brief Measurement vector type \f$p \times 1\f$ */
    using P_t = typename Base_class::P_t;               /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
    using R_t = typename Base_class::R_t;               /*!< \brief Measurement noise covariance matrix type \f$p \times p\f$ */
    using Q_t = typename Base_class::Q_t;               /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
    using statecov_t = typename Base_class::statecov_t; /*!< \brief Helper struct for passing around the state and its covariance in one variable */

    using A_t = typename Base_class::A_t; /*!< \brief System transition matrix type \f$n \times n\f$ */
    using B_t = typename Base_class::B_t; /*!< \brief Input to state mapping matrix type \f$n \times m\f$ */
    using H_t = typename Base_class::H_t; /*!< \brief State to measurement mapping matrix type \f$p \times n\f$ */
    using K_t = typename Base_class::K_t; /*!< \brief Kalman gain matrix type \f$n \times p\f$ */
    //}

  public:
    /*!
     * \brief Convenience default constructor.
     *
     * This constructor should not be used if applicable. If used, the main constructor has to be called afterwards,
     * before using this class, otherwise the object is invalid (not initialized).
     */
    NCLKF(){};

    /*!
     * \brief The main constructor.
     *
     * \param A                         State transition matrix of the system (n x n).
     * \param B                         Input to state mapping matrix of the system (n x m).
     * \param H                         State to measurement mapping matrix of the system (p x n).
     * \param l                         The norm constraint, applied to the specified states.
     */
    NCLKF(const A_t& A, const B_t& B, const H_t& H, const double l) : Base_class(A, B, H), l(l){};

  protected:
    double l;

  protected:
    /* computeKalmanGain() method //{ */
    virtual K_t computeKalmanGain(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const override
    {
      const R_t W = H * sc.P * H.transpose() + R;
      const R_t W_inv = Base_class::invert_W(W);
      const K_t K_orig = sc.P * H.transpose() * W_inv;

      const z_t inn = z - (H * sc.x);  // innovation
      const x_t x = sc.x + K_orig * inn;
      const double inn_scale = inn.transpose() * W_inv * inn;

      const double x_norm = x.norm();
      const K_t K = K_orig + (l / x_norm - 1.0) * x * (inn.transpose() * W_inv) / inn_scale;

      return K;
    }
    //}
  };
  //}

  /* class NCLKF_partial //{ */

  /**
   * \brief This class implements the partially norm-constrained linear Kalman filter \cite NCLKF.
   *
   * Note that the class is templated. The template parameters specify the number of states,
   * number of inputs and number of measurements of the system. The last template parameter
   * specifies the number of states which are norm-constrained.
   *
   * The norm constraint is specified in the constructor together with the indices of the
   * states to which the constraint applies.
   *
   * Example usage:
   * \include test/nckf/test.cpp
   *
   */
  template <int n_states, int n_inputs, int n_measurements, int n_norm_constrained_states>
  class NCLKF_partial : public NCLKF<n_states, n_inputs, n_measurements>
  {
  public:
    /* NCLKF_partial definitions (typedefs, constants etc) //{ */
    static const int n = n_states;                   /*!< \brief Length of the state vector of the system. */
    static const int m = n_inputs;                   /*!< \brief Length of the input vector of the system. */
    static const int p = n_measurements;             /*!< \brief Length of the measurement vector of the system. */
    static const int nq = n_norm_constrained_states; /*!< \brief Number of states to which the norm constraint applies. */
    using Base_class = NCLKF<n, m, p>;               /*!< \brief Base class of this class. */

    using x_t = typename Base_class::x_t;               /*!< \brief State vector type \f$n \times 1\f$ */
    using u_t = typename Base_class::u_t;               /*!< \brief Input vector type \f$m \times 1\f$ */
    using z_t = typename Base_class::z_t;               /*!< \brief Measurement vector type \f$p \times 1\f$ */
    using P_t = typename Base_class::P_t;               /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
    using R_t = typename Base_class::R_t;               /*!< \brief Measurement noise covariance matrix type \f$p \times p\f$ */
    using Q_t = typename Base_class::Q_t;               /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
    using statecov_t = typename Base_class::statecov_t; /*!< \brief Helper struct for passing around the state and its covariance in one variable */

    using A_t = typename Base_class::A_t; /*!< \brief System transition matrix type \f$n \times n\f$ */
    using B_t = typename Base_class::B_t; /*!< \brief Input to state mapping matrix type \f$n \times m\f$ */
    using H_t = typename Base_class::H_t; /*!< \brief State to measurement mapping matrix type \f$p \times n\f$ */
    using K_t = typename Base_class::K_t; /*!< \brief Kalman gain matrix type \f$n \times p\f$ */

    using indices_t = std::array<int, nq>;     /*!< \brief Indices of the norm-constrained states type */
    using xq_t = Eigen::Matrix<double, nq, 1>; /*!< \brief Norm-constrained states vector type \f$ n_q \times 1 \f$ */
    using Hq_t = Eigen::Matrix<double, p, nq>; /*!< \brief Norm-constrained measurement mapping type \f$ p \times n_q \f$ */
    using Kq_t = Eigen::Matrix<double, nq, p>; /*!< \brief Norm-constrained kalman gain type \f$ n_q \times p \f$ */
    //}

  public:
    /*!
     * \brief Convenience default constructor.
     *
     * This constructor should not be used if applicable. If used, the main constructor has to be called afterwards,
     * before using this class, otherwise the object is invalid (not initialized).
     */
    NCLKF_partial(){};

    /*!
     * \brief The main constructor.
     *
     * \param A                         State transition matrix of the system (n x n).
     * \param B                         Input to state mapping matrix of the system (n x m).
     * \param H                         State to measurement mapping matrix of the system (p x n).
     * \param l                         The norm constraint, applied to the specified states.
     * \param norm_constrained_indices  Indices of the norm-constrained states in the state vector.
     */
    NCLKF_partial(const A_t& A, const B_t& B, const H_t& H, const double l, const indices_t& norm_constrained_indices)
        : Base_class(A, B, H, l), norm_indices(norm_constrained_indices)
    {
      if (n >= 0)
      {
        for (auto& idx : norm_indices)
        {
          if (idx > n)
          {
            printf("[NCLKF_partial]: Index of a norm-constrained state (%d) cannot be higher than the number of states (%d)! Setting it to zero.", idx, n);
            idx = 0;
          }
          if (idx < 0)
          {
            printf("[NCLKF_partial]: Index of a norm-constrained state (%d) cannot be less than zero! Setting it to zero.", idx);
            idx = 0;
          }
        }
      }
    };

  private:
    indices_t norm_indices;

    /* helper methods for Eigen matrix subscripting //{ */

    template <typename T, int rows, int cols, size_t out_rows, size_t out_cols>
    Eigen::Matrix<T, out_rows, out_cols> select_indices(const Eigen::Matrix<T, rows, cols>& mat, const std::array<int, out_rows>& row_indices,
                                                        const std::array<int, out_cols>& col_indices) const
    {
      Eigen::Matrix<T, out_rows, cols> tmp;
      for (int it = 0; it < (int)out_rows; it++)
      {
        const auto row = row_indices[it];
        assert(rows < 0 || row < rows);
        tmp.row(it) = mat.row(row);
      }

      Eigen::Matrix<T, out_rows, out_cols> ret;
      for (int it = 0; it < (int)out_cols; it++)
      {
        const auto col = col_indices[it];
        assert(cols < 0 || col < cols);
        tmp.col(it) = tmp.col(col);
      }

      return ret;
    }

    template <typename T, int rows, int cols, size_t out_rows>
    Eigen::Matrix<T, out_rows, cols> select_rows(const Eigen::Matrix<T, rows, cols>& mat, const std::array<int, out_rows>& row_indices) const
    {
      Eigen::Matrix<T, out_rows, cols> ret;
      for (int it = 0; it < (int)out_rows; it++)
      {
        const auto row = row_indices[it];
        assert(rows < 0 || row < rows);
        ret.row(it) = mat.row(row);
      }
      return ret;
    }

    template <typename T, int rows, int cols, size_t out_cols>
    Eigen::Matrix<T, rows, out_cols> select_cols(const Eigen::Matrix<T, rows, cols>& mat, const std::array<int, out_cols>& col_indices) const
    {
      Eigen::Matrix<T, rows, out_cols> ret;
      for (int it = 0; it < (int)out_cols; it++)
      {
        const auto col = col_indices[it];
        assert(cols < 0 || col < cols);
        ret.col(it) = mat.col(col);
      }
      return ret;
    }

    template <typename T, int rows, size_t out_rows>
    Eigen::Matrix<T, out_rows, 1> select_indices(const Eigen::Matrix<T, rows, 1>& vec, const std::array<int, out_rows>& row_indices) const
    {
      return select_rows(vec, row_indices);
    }

    template <typename T, int rows, int cols, size_t n_rows>
    void set_rows(const Eigen::Matrix<T, (size_t)n_rows, cols>& from_mat, Eigen::Matrix<T, rows, cols>& to_mat,
                  const std::array<int, n_rows>& row_indices) const
    {
      for (int rit = 0; rit < (int)n_rows; rit++)
      {
        const auto ridx = row_indices.at(rit);
        assert(rows < 0 || ridx < rows);
        to_mat.row(ridx) = from_mat.row(rit);
      }
    }

    //}

  protected:
    /* computeKalmanGain() method //{ */
    virtual K_t computeKalmanGain(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const override
    {
      const R_t W = H * sc.P * H.transpose() + R;
      const R_t W_inv = Base_class::invert_W(W);
      K_t K = sc.P * H.transpose() * W_inv;

      // calculate the kalman gain for the norm-constrained states
      {
        const Kq_t K_orig = select_rows(K, norm_indices);
        const xq_t xq = select_indices(sc.x, norm_indices);
        const Hq_t Hq = select_cols(H, norm_indices);
        const z_t inn = z - (Hq * xq);  // innovation
        const xq_t x = xq + K_orig * inn;
        const double inn_scale = inn.transpose() * W_inv * inn;

        const double x_norm = x.norm();
        const Kq_t Kq = K_orig + (Base_class::l / x_norm - 1.0) * x * (inn.transpose() * W_inv) / inn_scale;
        set_rows(Kq, K, norm_indices);
      }

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

    using X_t = typename Base_class::X_t;      // state sigma points matrix n*w
    using Z_t = typename Base_class::Z_t;      // measurement sigma points matrix p*w
    using Pzz_t = typename Base_class::Pzz_t;  // Pzz helper matrix p*n
    using K_t = typename Base_class::K_t;      // kalman gain n*p
    //}

  public:
    NCUKF(){};

    NCUKF(const transition_model_t& transition_model, const observation_model_t& observation_model, const double l, const double alpha = 1e-3,
          const double kappa = 1, const double beta = 2)
        : Base_class(transition_model, observation_model, alpha, kappa, beta), l(l){};

  public:
    /* correct() method //{ */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R) const override
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
      const z_t inn = (z - z_exp);  // innovation
      const K_t K = computeKalmanGain(sc.x, inn, Pxz, Pzz);

      // check whether the inverse produced valid numbers
      if (!K.array().isFinite().all())
      {
        printf("UKF: inverting of Pzz in correction update produced non-finite numbers!!! Fix your covariances (the measurement's is probably too low...)");
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
    double l;

  protected:
    /* computeKalmanGain() method //{ */
    virtual K_t computeKalmanGain(const x_t& x, const z_t& inn, const K_t& Pxz, const Pzz_t& Pzz) const override
    {
      const Pzz_t Pzz_inv = Base_class::computeInverse(Pzz);
      const K_t K_orig = Pxz * Pzz_inv;

      const x_t x_pred = x + K_orig * inn;
      const double inn_scale = inn.transpose() * Pzz_inv * inn;

      const double x_norm = x_pred.norm();
      const K_t K = K_orig + (l / x_norm - 1.0) * x_pred * (inn.transpose() * Pzz_inv) / inn_scale;

      return K;
    }
    //}
  };
  //}

}  // namespace mrs_lib

#endif  // NCKFSYSTEMMODELS_H

