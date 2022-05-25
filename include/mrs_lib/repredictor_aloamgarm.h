#ifndef REPREDICTOR_ALOAMGARM_H
#define REPREDICTOR_ALOAMGARM_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Time.h>
#include <functional>
#include <ros/ros.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/repredictor.h>

namespace mrs_lib
{
/**
 * \brief Implementation of the RepredictorAloamgarm for fusing measurements with variable delays.
 *
 * A standard state-space system model is assumed for the repredictor with system inputs and measurements,
 * generated from the system state vector. The inputs and measurements may be delayed with varying durations (an older measurement may become available after a
 * newer one). A typical use-case scenario is when you have one "fast" but imprecise sensor and one "slow" but precise sensor and you want to use them both to
 * get a good prediction (eg. height from the Garmin LiDAR, which comes at 100Hz, and position from a SLAM, which comes at 10Hz with a long delay). If the slow
 * sensor is significantly slower than the fast one, fusing its measurement at the time it arrives without taking into account the sensor's delay may
 * significantly bias your latest estimate.
 *
 * To accomodate this, the RepredictorAloamgarm keeps a buffer of N last inputs and measurements (N is specified
 * in the constructor). This buffer is then used to re-predict the desired state to a specific time, as
 * requested by the user. Note that the re-prediction is evaluated in a lazy manner only when the user requests it,
 * so it goes through the whole history buffer every time a prediction is requested.
 *
 * The RepredictorAloamgarm utilizes a fusion Model (specified as the template parameter), which should implement
 * the predict() and correct() methods. This Model is used for fusing the system inputs and measurements
 * as well as for predictions. Typically, this Model will be some kind of a Kalman Filter (LKF, UKF etc.).
 * \note The Model should be able to accomodate predictions with varying time steps in order for
 * the RepredictorAloamgarm to work correctly (see eg. the varstepLKF class).
 *
 * \tparam Model the prediction and correction model (eg. a Kalman Filter).
 *
 */
template <class Model>
class RepredictorAloamgarm : public Repredictor<Model> {
public:
  /* states, inputs etc. definitions (typedefs, constants etc) //{ */

  using x_t        = typename Model::x_t;             /*!< \brief State vector type \f$n \times 1\f$ */
  using u_t        = typename Model::u_t;             /*!< \brief Input vector type \f$m \times 1\f$ */
  using z_t        = typename Model::z_t;             /*!< \brief Measurement vector type \f$p \times 1\f$ */
  using P_t        = typename Model::P_t;             /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
  using R_t        = typename Model::R_t;             /*!< \brief Measurement noise covariance matrix type \f$p \times p\f$ */
  using Q_t        = typename Model::Q_t;             /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
  using statecov_t = typename Model::statecov_t;      /*!< \brief Helper struct for passing around the state and its covariance in one variable */
  using ModelPtr   = typename std::shared_ptr<Model>; /*!< \brief Shorthand type for a shared pointer-to-Model */
  using history_t  = typename Repredictor<Model>::history_t;

  //}

public:
  /* constructor //{ */

  /*!
   * \brief Variation of the constructor for kalman filter using nis_buffer (ALOAMGARM)
   *
   * Initializes the RepredictorAloamgarm with the necessary initial and default values.
   *
   * \param x0             Initial state.
   * \param P0             Covariance matrix of the initial state uncertainty.
   * \param u0             Initial system input.
   * \param Q0             Default covariance matrix of the process noise.
   * \param t0             Time stamp of the initial state.
   * \param model          Default prediction and correction model.
   * \param hist_len       Length of the history buffer for system inputs and measurements.
   * \param nis_buffer     Circular buffer for NIS values.
   */
  RepredictorAloamgarm(const x_t& x0, const P_t& P0, const u_t& u0, const Q_t& Q0, const ros::Time& t0, const ModelPtr& model, const unsigned hist_len,
                       const std::shared_ptr<boost::circular_buffer<double>>& nis_buffer) {
    Repredictor<Model>::m_sc            = {x0, P0, nis_buffer};
    Repredictor<Model>::m_default_model = model;
    Repredictor<Model>::m_history       = history_t(hist_len);
    assert(hist_len > 0);
    Repredictor<Model>::addInputChangeWithNoise(u0, Q0, t0, model);
  };
  //}
};
}  // namespace mrs_lib


#endif  // REPREDICTOR_ALOAMGARM_H
