// clang: MatousFormat
/**  \file JLKF
     \brief Defines JLKF
     \author Vaclav Pritzl - vaclav.pritzl@fel.cvut.cz
 */
#ifndef JLKF_H
#define JLKF_H

#include <mrs_lib/kalman_filter.h>
#include <mrs_lib/lkf.h>
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
  class JLKF : public varstepLKF<n_states, n_inputs, n_measurements>
  {
  public:
    /* LKF definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;
    using Base_class = varstepLKF<n, m, p>;

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
    using K_t = typename Base_class::K_t;  // Kalman gain

    typedef Eigen::Matrix<double, p, p> C_t; /*!< \brief correntropy gain \f$p \times p\f$ */
    typedef Eigen::Matrix<double, n, n> D_t; /*!< \brief D \f$n \times n\f$ */

    using generateA_t = std::function<A_t(double)>;
    using generateB_t = std::function<B_t(double)>;
    //}

  public:
    /*!
     * \brief The main constructor.
     *
     * \param generateA a function, which returns the state transition matrix \p A based on the time difference \p dt.
     * \param generateB a function, which returns the input to state mapping matrix \p B based on the time difference \p dt.
     * \param H         the state to measurement mapping matrix.
     */
    JLKF(const generateA_t& generateA, const generateB_t& generateB, const H_t& H, const double& sigma, const ros::NodeHandle& nh, const double& nis_thr,
          const double& nis_avg_thr)
        : varstepLKF<n, m, p>(generateA, generateB, H),
          m_generateA(generateA),
          m_generateB(generateB),
          m_sigma(sigma),
          m_nh(nh),
          m_nis_thr(nis_thr),
          m_nis_avg_thr(nis_avg_thr)
    {
      /* std::cout << "Creating jlkf" << std::endl; */
      Base_class::H = H;
      debug_nis_pub = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_nis", 1);
      debug_residual_pub = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_residual", 1);
      debug_tmp_pub = m_nh.advertise<mrs_msgs::BoolStamped>("debug_crosscov", 1);
      debug_meas_pub = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_meas", 1);
      debug_cov_diff_pub = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_cov_diff", 1);
      debug_cov_pub = m_nh.advertise<mrs_msgs::Float64ArrayStamped>("debug_cov_jlkf", 1);
      debug_meas_jump_pub = m_nh.advertise<mrs_msgs::BoolStamped>("debug_meas_jump", 1);
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
      ret.x = Base_class::state_predict(A, sc.x, B, u);
      ret.P = Base_class::covariance_predict(A, sc.P, Q, dt);
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
      return correction_impl(sc, z, R, Base_class::H);
    };
    //}

  protected:
    /* invert_W() method //{ */
    /* static R_t invert_W(R_t W) */
    /* { */
    /*   Eigen::ColPivHouseholderQR<R_t> qr(W); */
    /*   if (!qr.isInvertible()) */
    /*   { */
    /*     // add some stuff to the tmp matrix diagonal to make it invertible */
    /*     R_t ident = R_t::Identity(W.rows(), W.cols()); */
    /*     W += 1e-9 * ident; */
    /*     qr.compute(W); */
    /*     if (!qr.isInvertible()) */
    /*     { */
    /*       // never managed to make this happen except for explicitly putting NaNs in the input */
    /*       throw inverse_exception(); */
    /*     } */
    /*   } */
    /*   const R_t W_inv = qr.inverse(); */
    /*   return W_inv; */
    /* } */
    //}


    /* computeKalmanGain() method //{ */
    virtual K_t computeKalmanGain(const statecov_t& sc, [[maybe_unused]] const z_t& z, const R_t& R, const H_t& H, double& nis, H_t& H_out,
                                  const double& nis_thr, const double& nis_avg_thr) const
    {
      // TODO return the parameters back to const
      H_out = H;
      // calculation of the kalman gain K
      /* const R_t W = H * sc.P * H.transpose() + R; */
      /* const R_t W_inv = Base_class::invert_W(W); */
      /* K_t K = sc.P * H.transpose() * W_inv; */
      /* const z_t y = z - (H * sc.x); */

      R_t W = H * sc.P * H.transpose() + R;
      R_t W_inv = Base_class::invert_W(W);
      K_t K = sc.P * H.transpose() * W_inv;
      z_t y = z - (H * sc.x);
      if (y(0, 0) > 100)
      {
        std::cout << "y: " << y << ", z: " << z << ", H: " << H << ", sc.x: " << sc.x << std::endl;
      }

      if (H(0, 0) > 0)
      {
        mrs_msgs::Float64ArrayStamped msg_res;
        msg_res.header.stamp = sc.stamp;
        for (int i = 0; i < y.rows(); i++)
        {
          msg_res.values.push_back(y(i, 1));
        }
        debug_residual_pub.publish(msg_res);
      }

      /* const double nis = (y.transpose() * W_inv * y)(0, 0); */
      nis = (y.transpose() * W_inv * y)(0, 0);
      double nis_avg = 0;
      int count = 0;
      bool nis_over_thr = false;

      double nis_thr_tmp = nis_thr;

      if (H(0, 0) > 0 && H(0, 3) != 0)
      {
        // measurement jump
        /* mrs_msgs::BoolStamped msg_jump; */
        /* msg_jump.stamp = sc.stamp; */
        /* if (sc.measurement_jumped) */
        /* { */
        /*   msg_jump.data = true; */
        /* } else */
        /* { */
        /*   msg_jump.data = false; */
        /* } */
        /* debug_meas_jump_pub.publish(msg_jump); */

        mrs_msgs::Float64ArrayStamped msg_meas;
        msg_meas.header.stamp = sc.stamp;
        msg_meas.values.push_back(z(0, 0));
        debug_meas_pub.publish(msg_meas);

        mrs_msgs::Float64ArrayStamped msg;
        msg.header.stamp = sc.stamp;

        msg.values.push_back(nis);

        if (nis == 0)
        {
          std::cout << "nula, nis: " << nis << ", y: " << y << ", W_inv: " << W_inv << std::endl;
        }

        if (sc.nis_buffer != nullptr)
        {
          sc.nis_buffer->push_back(nis);
          for (auto it = sc.nis_buffer->begin(); it != sc.nis_buffer->end(); it++)
          {
            nis_avg += *it;
            count++;
            if (*it > nis_thr_tmp)
            {
              nis_over_thr = true;

            }
          }
          if (count > 0)
          {
            nis_avg /= count;
          }
          msg.values.push_back(nis_avg);
        }
        /* if (nis > 3.9e-5) */
        /* if (H(0, 0) > 0 && count > 0 && nis_avg > 3.9e-5 / count) */
        /* if (nis > nis_thr || (count > 0 && nis_avg > nis_avg_thr)) */
        /* if (nis > nis_thr || (nis_over_thr && sc.measurement_jumped)) */
        /* if (nis > nis_thr_tmp || (nis_over_thr) || sc.measurement_jumped) */
        /* if (nis > nis_thr_tmp || nis_over_thr) */
        /* if (nis > nis_thr_tmp || sc.measurement_jumped) */
        if (nis > nis_thr)
        /* if ((count > 0 && nis_avg > 3.9e-5)) */
        {
          // old jump correction
          K = K.Zero();
          A_t mask = mask.Zero();
          for (int i = n_states - n_biases; i < n_states; i++)
          {
            mask(i, i) = 1;
          }
          const H_t H_bias_only = H * mask;
          K = H_bias_only.transpose() * Base_class::invert_W(H_bias_only * H_bias_only.transpose());
          H_out = H_bias_only;
        }

        /* for (int i = 0; i < K.size(); i++) */
        /* { */
        /*   if (abs(K(i)) < 1e-30) */
        /*   { */
        /*     K(i) = 0; */
        /*   } */
        /* } */
        debug_nis_pub.publish(msg);
      }

      K_t test = H.transpose() * Base_class::invert_W(H * H.transpose());

      return K;
    }
    //}

    /* correction_impl() method //{ */
    template <int check = n>
    typename std::enable_if<check >= 0, statecov_t>::type correction_impl(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const
    {

      /* std::cout << "correction, sc.x: " << sc.x << std::endl; */

      // the correction phase
      statecov_t ret;
      double nis = -1;
      H_t H_out;
      mrs_msgs::Float64ArrayStamped msg_cov;
      msg_cov.header.stamp = sc.stamp;
      for (int i = 0; i < sc.P.size(); i++)
      {
        msg_cov.values.push_back(sc.P(i));
      }
      debug_cov_pub.publish(msg_cov);
      const K_t K = computeKalmanGain(sc, z, R, H, nis, H_out, m_nis_thr, m_nis_avg_thr);
      ret.x = sc.x + K * (z - (H * sc.x));
      ret.P = (P_t::Identity() - (K * H_out)) * sc.P;
      if (abs(ret.x(0)) > 100)
      {
        std::cout << "ret.x: " << ret.x << ", sc.x: " << sc.x << ", z: " << z << ", H: " << H << ", K: " << K << ", sc.P: " << sc.P << std::endl;
      }
      ret.nis_buffer = sc.nis_buffer;
      return ret;
    }

    template <int check = n>
        typename std::enable_if < check<0, statecov_t>::type correction_impl(const statecov_t& sc, const z_t& z, const R_t& R, const H_t& H) const
    {
      // the correction phase
      // THIS IS NOT USED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1111
      statecov_t ret;
      double nis = -1;
      H_t H_out;
      const K_t K = computeKalmanGain(sc, z, R, H, nis, H_out, m_nis_thr, m_nis_avg_thr);
      ret.x = sc.x + K * (z - (H * sc.x));
      ret.P = (P_t::Identity() - (K * H)) * sc.P;
      ret.nis_buffer = sc.nis_buffer;
      return ret;
    }
    //}

  private:
    double m_sigma;
    ros::NodeHandle m_nh;
    ros::Publisher debug_nis_pub;
    ros::Publisher debug_residual_pub;
    ros::Publisher debug_tmp_pub;
    ros::Publisher debug_cov_diff_pub;
    ros::Publisher debug_cov_pub;
    ros::Publisher debug_meas_pub;
    ros::Publisher debug_meas_jump_pub;
    std::vector<double> m_nis_window;
    double m_nis_thr;
    double m_nis_avg_thr;


  private:
    generateA_t m_generateA;
    generateB_t m_generateB;
  };  // namespace mrs_lib
  /*//}*/


}  // namespace mrs_lib

#endif  // JLKF_H
