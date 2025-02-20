// clang: MatousFormat
/**  \file
     \brief Defines KalmanFilter - an abstract class, defining common interfaces and types for a generic Kalman filter.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef SYSTEMMODEL_H
#define SYSTEMMODEL_H

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace mrs_lib
{
  /* KalmanFilter virtual class //{ */
  /**
   * \brief This abstract class defines common interfaces and types for a generic Kalman filter.
   *
   * \tparam n_states         number of states of the system (length of the \f$ \mathbf{x} \f$ vector).
   * \tparam n_inputs         number of inputs of the system (length of the \f$ \mathbf{u} \f$ vector).
   * \tparam n_measurements   number of measurements of the system (length of the \f$ \mathbf{z} \f$ vector).
   *
   */
  template <int n_states, int n_inputs, int n_measurements>
  class KalmanFilter
  {
  public:
    /* states, inputs etc. definitions (typedefs, constants etc) //{ */
    static const int n = n_states;       /*!< \brief Length of the state vector of the system. */
    static const int m = n_inputs;       /*!< \brief Length of the input vector of the system. */
    static const int p = n_measurements; /*!< \brief Length of the measurement vector of the system. */

    typedef Eigen::Matrix<double, n, 1> x_t; /*!< \brief State vector type \f$n \times 1\f$ */
    typedef Eigen::Matrix<double, m, 1> u_t; /*!< \brief Input vector type \f$m \times 1\f$ */
    typedef Eigen::Matrix<double, p, 1> z_t; /*!< \brief Measurement vector type \f$p \times 1\f$ */

    typedef Eigen::Matrix<double, n, n> P_t; /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
    typedef Eigen::Matrix<double, p, p> R_t; /*!< \brief Measurement noise covariance matrix type \f$p \times p\f$ */
    typedef Eigen::Matrix<double, n, n> Q_t; /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
    //}

    /* statecov_t struct //{ */
    /*!
     * \brief Helper struct for passing around the state and its covariance in one variable
     */
    struct statecov_t
    {
      x_t x; /*!< \brief State vector. */
      P_t P; /*!< \brief State covariance matrix. */
      rclcpp::Time stamp = rclcpp::Time(0);
    };
    //}

  public:
    /*!
     * \brief Applies the correction (update, measurement, data) step of the Kalman filter.
     *
     * This method applies the correction step to the state and covariance passed in \p sc using the measurement
     * \p z and measurement noise \p R. An optional parameter \p param may be used by some implementations, but it is
     * usually ignored. The updated state and covariance after the correction step is returned.
     *
     * \param sc          The state and covariance to which the correction step is to be applied.
     * \param z           The measurement vector to be used for correction.
     * \param R           The measurement noise covariance matrix to be used for correction.
     * \return            The state and covariance after the correction update.
     */
    virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R) const = 0;

    /*!
     * \brief Applies the prediction (time) step of the Kalman filter.
     *
     * This method applies the prediction step to the state and covariance passed in \p sc using the input \p u
     * and process noise \p Q. The state and covariance are updated by \p dt into the future, if applicable to the
     * implementation. An optional parameter \p param may be used by some implementations, but it is
     * usually ignored. The updated state and covariance after the prediction step is returned.
     *
     * \param sc          The state and covariance to which the prediction step is to be applied.
     * \param u           The input vector to be used for prediction.
     * \param Q           The process noise covariance matrix to be used for prediction.
     * \param dt          The time step for the prediction update (the state and covariance will be predicted by dt into the future).
     * \return            The state and covariance after the prediction step.
     */
    virtual statecov_t predict(const statecov_t& sc, const u_t& u, const Q_t& Q, double dt) const = 0;
  };
  //}
}  // namespace mrs_lib

#endif  // SYSTEMMODEL_H
