// clang: MatousFormat
/**  \file JLKF
     \brief Defines JLKF
     \author Vaclav Pritzl - vaclav.pritzl@fel.cvut.cz
 */
#ifndef JLKF_H
#define JLKF_H

#include <mrs_lib/kalman_filter_aloamgarm.h>
/* #include <mrs_lib/lkf.h> */
#include <iostream>

#include <vector>

// for debug
#include <ros/ros.h>
#include <mrs_msgs/Float64ArrayStamped.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/BoolStamped.h>

namespace mrs_lib
{

  /* class JLKF */ /*//{*/
  template <int n_states, int n_inputs, int n_measurements, int n_biases>
  class JLKF : public KalmanFilterAloamGarm<n_states, n_inputs, n_measurements>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;
    using Base_class = KalmanFilterAloamGarm<n, m, p>;

    using x_t = typename Base_class::x_t;
    using u_t = typename Base_class::u_t;
    using z_t = typename Base_class::z_t;
    using P_t = typename Base_class::P_t;
    using R_t = typename Base_class::R_t;
    using statecov_t = typename Base_class::statecov_t;
    /* using A_t = typename Base_class::A_t;  // measurement mapping p*n */
    /* using B_t = typename Base_class::B_t;  // process covariance n*n */
    /* using H_t = typename Base_class::H_t;  // measurement mapping p*n */
    using Q_t = typename Base_class::Q_t;  // process covariance n*n
    /* using K_t = typename Base_class::K_t;  // Kalman gain */

    typedef Eigen::Matrix<double, n, n> A_t;  /*!< \brief System transition matrix type \f$n \times n\f$ */
    typedef Eigen::Matrix<double, n, m> B_t;  /*!< \brief Input to state mapping matrix type \f$n \times m\f$ */
    typedef Eigen::Matrix<double, p, n> H_t;  /*!< \brief State to measurement mapping matrix type \f$p \times n\f$ */
    typedef Eigen::Matrix<double, n, p> K_t;  /*!< \brief Kalman gain matrix type \f$n \times p\f$ */

    typedef Eigen::Matrix<double, p, p> C_t; /*!< \brief correntropy gain \f$p \times p\f$ */
    typedef Eigen::Matrix<double, n, n> D_t; /*!< \brief D \f$n \times n\f$ */

    using generateA_t = std::function<A_t(double)>;
    using generateB_t = std::function<B_t(double)>;
    //}
    
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
     * \brief The main constructor.
     *
     * \param generateA a function, which returns the state transition matrix \p A based on the time difference \p dt.
     * \param generateB a function, which returns the input to state mapping matrix \p B based on the time difference \p dt.
     * \param H         the state to measurement mapping matrix.
     */
    JLKF(const generateA_t& generateA, const generateB_t& generateB, const H_t& H, const ros::NodeHandle& nh, const double& nis_thr,
         const double& nis_avg_thr)
        : m_generateA(generateA),
          m_generateB(generateB),
          H(H),
          m_nh(nh),
          m_nis_thr(nis_thr),
          m_nis_avg_thr(nis_avg_thr)
    {
      debug_nis_pub = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_nis", 1);
    };

    /* predict() method //{ */
    /*!
     * \brief Applies the prediction (time) step of the Kalman filter.
     *
     * This method applies the linear Kalman filter prediction step to the state and covariance
     * passed in \p sc using the input \p u and process noise \p Q. The process noise covariance
     * \p Q is scaled by the \p dt parameter. The updated state and covariance after
     * the prediction step is returned.
     *
     * \param sc          The state and covariance to which the prediction step is to be applied.
     * \param u           The input vector to be used for prediction.
     * \param Q           The process noise covariance matrix to be used for prediction.
     * \param dt          Used to scale the process noise covariance \p Q and to generate the state transition and input to state mapping matrices \p A and \B
     * using the functions, passed in the object's constructor. \return            The state and covariance after the prediction step.
     *
     * \note Note that the \p dt parameter is used to scale the process noise covariance \p Q and to generate the system matrices #A or #B using the functions,
     * passed in the constructor!
     */
    virtual statecov_t predict(const statecov_t& sc, const u_t& u, const Q_t& Q, double dt) const override
    {
      /* std::cout << "prediction, sc.x: " << sc.x << std::endl; */
      statecov_t ret;
      A_t A = m_generateA(dt);
      B_t B = m_generateB(dt);
      ret.x = state_predict(A, sc.x, B, u);
      ret.P = covariance_predict(A, sc.P, Q, dt);
      ret.nis_buffer = sc.nis_buffer;
      return ret;
    };
    //}

    /* correct() method //{ */
    /*!
     * \brief Applies the correction (update, measurement, data) step of the Kalman filter.
     *
     * This method applies the linear Kalman filter correction step to the state and covariance
     * passed in \p sc using the measurement \p z and measurement noise \p R. The updated state
     * and covariance after the correction step is returned.
     *
     * \param sc          The state and covariance to which the correction step is to be applied.
     * \param z           The measurement vector to be used for correction.
     * \param R           The measurement noise covariance matrix to be used for correction.
     * \return            The state and covariance after the correction update.
     */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R) const override
    {
      /* return correct_optimized(sc, z, R, H); */
      return correction_impl(sc, z, R, H);
    };
    //}

  protected:
    /* invert_W() method //{ */
    static R_t invert_W(R_t W)
    {
      Eigen::ColPivHouseholderQR<R_t> qr(W);
      if (!qr.isInvertible())
      {
        // add some stuff to the tmp matrix diagonal to make it invertible
        R_t ident = R_t::Identity(W.rows(), W.cols());
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
    virtual K_t computeKalmanGain(const statecov_t& sc, [[maybe_unused]] const z_t& z, const R_t& R, const H_t& H, double& nis, H_t& H_out,
                                  const double& nis_thr, const double& nis_avg_thr) const
    {
      H_out = H;

      R_t W = H * sc.P * H.transpose() + R;
      R_t W_inv = invert_W(W);
      K_t K = sc.P * H.transpose() * W_inv;
      z_t y = z - (H * sc.x);

      /* const double nis = (y.transpose() * W_inv * y)(0, 0); */
      nis = (y.transpose() * W_inv * y)(0, 0);
      double nis_avg = 0;
      int count = 0;
      bool nis_over_thr = false;

      double nis_thr_tmp = nis_thr;

      if (H(0, 0) > 0 && H(0, 3) != 0)
      /* if (H(0, 0) > 0) */
      {
        mrs_msgs::Float64ArrayStamped msg;
        msg.header.stamp = sc.stamp;

        msg.values.push_back(nis);

        if (sc.nis_buffer != nullptr)
        {

          sc.nis_buffer->push_back(nis);
          for (auto it = sc.nis_buffer->begin(); it != sc.nis_buffer->end(); it++)
          {
            nis_avg += *it;
            count++;
            if (*it > nis_thr_tmp)
            {
              nis_thr_tmp /= 10;
            }
          }
          if (count > 0)
          {
            nis_avg /= count;
          }
          msg.values.push_back(nis_avg);
          msg.values.push_back(nis_thr_tmp);
        }
        if (nis > nis_thr_tmp)
        {
          // old jump correction
          K = K.Zero();
          A_t mask = mask.Zero();
          for (int i = n_states - n_biases; i < n_states; i++)
          {
            mask(i, i) = 1;
          }
          const H_t H_bias_only = H * mask;
          K = H_bias_only.transpose() * invert_W(H_bias_only * H_bias_only.transpose());
          H_out = H_bias_only;
        }

        debug_nis_pub.publish(msg);
      }

      K_t test = H.transpose() * invert_W(H * H.transpose());

      return K;
    }
    //}

    /* correction_impl() method //{ */
    template <int check = n>
    typename std::enable_if<check >= 0, statecov_t>::type correction_impl(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const
    {
      // the correction phase
      statecov_t ret;
      double nis = -1;
      H_t H_out;
      const K_t K = computeKalmanGain(sc, z, R, H, nis, H_out, m_nis_thr, m_nis_avg_thr);
      ret.x = sc.x + K * (z - (H * sc.x));
      ret.P = (P_t::Identity() - (K * H_out)) * sc.P;
      ret.nis_buffer = sc.nis_buffer;
      return ret;
    }

    template <int check = n>
        typename std::enable_if < check<0, statecov_t>::type correction_impl(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const
    {
      // the correction phase
      statecov_t ret;
      double nis = -1;
      H_t H_out;
      const K_t K = computeKalmanGain(sc, z, R, H, nis, H_out, m_nis_thr, m_nis_avg_thr);
      ret.x = sc.x + K * (z - (H * sc.x));
      ret.P = (P_t::Identity(sc.P.rows(), sc.P.cols()) - (K * H_out)) * sc.P;
      ret.nis_buffer = sc.nis_buffer;
      return ret;
    }
    //}

  private:
    ros::NodeHandle m_nh;
    ros::Publisher debug_nis_pub;
    std::vector<double> m_nis_window;
    double m_nis_thr;
    double m_nis_avg_thr;


  private:
    generateA_t m_generateA;
    generateB_t m_generateB;

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
    template <int check = n_inputs>
    static inline typename std::enable_if<check == 0, x_t>::type state_predict(const A_t& A, const x_t& x, [[maybe_unused]] const B_t& B,
                                                                               [[maybe_unused]] const u_t& u)
    {
      return A * x;
    }

    template <int check = n_inputs>
    static inline typename std::enable_if<check != 0, x_t>::type state_predict(const A_t& A, const x_t& x, const B_t& B, const u_t& u)
    {
      return A * x + B * u;
    }
    //}

  };  // namespace mrs_lib
  /*//}*/


}  // namespace mrs_lib

#endif  // JLKF_H
