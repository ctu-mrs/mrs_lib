#ifndef REPREDICTOR_H
#define REPREDICTOR_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Time.h>
#include <functional>
#include <mrs_lib/utils.h>

namespace mrs_lib
{
  /**
  * \brief Implementation of the Repredictor for fusing measurements with variable delays.
  *
  * A standard state-space system model is assumed for the repredictor with system inputs and measurements,
  * generated from the system state vector. The inputs and measurements may be delayed with varying durations.
  * To accomodate this, the Repredictor keeps a buffer of N last inputs and M last measurements (N and M are
  * specified in the constructor). This buffer is then used to re-predict the desired state at the time,
  * requested by the user.
  *
  * The Repredictor utilizes a fusion Model (specified as the template parameter), which should implement
  * the predict() and correct() methods. This Model is used for fusing the system inputs and measurements
  * as well as for predictions. Typically, this Model will be some kind of a Kalman Filter (LKF, UKF etc.).
  * \note The Model should be able to accomodate predictions with varying time steps in order for
  * the Repredictor to work correctly (see eg. the varstepLKF class).
  *
  * \tparam Model the prediction and correction model (eg. a Kalman Filter).
  *
  */
  template <class Model>
  class Repredictor
  {
  public:
    /* states, inputs etc. definitions (typedefs, constants etc) //{ */

    using x_t = typename Model::x_t;                  /*!< \brief State vector type \f$n \times 1\f$ */
    using u_t = typename Model::u_t;                  /*!< \brief Input vector type \f$m \times 1\f$ */
    using z_t = typename Model::z_t;                  /*!< \brief Measurement vector type \f$p \times 1\f$ */
    using P_t = typename Model::P_t;                  /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
    using R_t = typename Model::R_t;                  /*!< \brief Measurement noise covariance matrix type \f$p \times p\f$ */
    using Q_t = typename Model::Q_t;                  /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
    using statecov_t = typename Model::statecov_t;    /*!< \brief Helper struct for passing around the state and its covariance in one variable */
    using ModelPtr = typename std::shared_ptr<Model>; /*!< \brief Shorthand type for a shared pointer-to-Model */

    //}

    /* predictTo() method //{ */
  /*!
    * \brief Estimates the system state and covariance matrix at the specified time.
    *
    * The measurement and system input histories are used to estimate the state vector and
    * covariance matrix values at the specified time, which are returned.
    *
    * \param to_stamp   The desired time at which the state vector and covariance matrix should be estimated.
    * \return           Returns the estimated state vector and covariance matrix in a single struct.
    *
    */
    statecov_t predictTo(const ros::Time& to_stamp)
    {
      auto inpt_it = std::begin(m_inpt_hist);
      auto meas_it = std::begin(m_meas_hist);
      auto cur_stamp = inpt_it->stamp;
      auto cur_sc = m_sc;
      if (debug)
        std::cerr << "Rep: (" << m_inpt_hist.size() << " inputs, " << m_meas_hist.size() << " measurements)" << std::endl;
      do
      {
        ros::Time next_inpt_stamp = to_stamp;
        if ((inpt_it+1) != std::end(m_inpt_hist) && (inpt_it+1)->stamp < to_stamp)
          next_inpt_stamp = (inpt_it+1)->stamp;

        // decide whether we should predict to the next input or to the current measurement
        // (whichever is sooner according to the stamp)
        const bool meas_available = meas_it != std::end(m_meas_hist);
        const bool use_meas_next = meas_available && meas_it->stamp <= next_inpt_stamp;
        const auto next_stamp = use_meas_next ? meas_it->stamp : next_inpt_stamp;

        // do the prediction
        cur_sc = predictFrom(cur_sc, *inpt_it, cur_stamp, next_stamp);
        if (debug)
          std::cerr << "predict\t" << cur_stamp << "\t->\t" << next_stamp << std::endl;
        cur_stamp = next_stamp;
        if (use_meas_next)
        {
          cur_sc = correctFrom(cur_sc, *meas_it);
          if (debug)
            std::cerr << "correct\t" << cur_stamp << std::endl;
        }

        // increment either the input or measurement iterator depending on which we used
        if (use_meas_next)
        {
          if (meas_it != std::end(m_meas_hist))
            meas_it++;
        }
        else
        {
          if (inpt_it+1 != std::end(m_inpt_hist))
            inpt_it++;
        }
      }
      while (!(inpt_it == std::end(m_inpt_hist) && meas_it == std::end(m_meas_hist)) && cur_stamp < to_stamp);
      return cur_sc;
    }
    //}

    /* addInput() method //{ */
  /*!
    * \brief Adds one system input to the input history buffer, removing the oldest element in the buffer if it is full.
    *
    * \param u      The system input vector to be added.
    * \param Q      The process noise covariance matrix, corresponding to the input vector.
    * \param stamp  Time stamp of the input vector and covariance matrix.
    * \param model  Optional pointer to a specific Model to be used with this input (eg. mapping it to different states). If it equals to nullptr, the default model specified in the constructor will be used.
    *
    * \note The system input vector will not be added if it is older than the oldest one in the history buffer.
    *
    */
    void addInput(const u_t& u, const Q_t& Q, const ros::Time& stamp, const ModelPtr& model = nullptr)
    {
      const inpt_t inpt {u, Q, stamp, model};
      const auto next_inpt_it = std::lower_bound(std::begin(m_inpt_hist), std::end(m_inpt_hist), inpt, &Repredictor<Model>::earlier<inpt_t>);
      // check if adding a new input would throw out the oldest one
      if (m_inpt_hist.size() == m_inpt_hist.capacity())
      { // if so, first update m_sc
        // figure out, what will be the oldest input in the buffer after inserting the newly received one
        auto new_oldest_inpt = m_inpt_hist.front();
        if (m_inpt_hist.size() == 1
         || (m_inpt_hist.size() > 1 && stamp > m_inpt_hist.at(0).stamp && stamp < m_inpt_hist.at(1).stamp))
        {
          new_oldest_inpt = inpt;
        }
        else if (m_inpt_hist.size() > 1)
        {
          new_oldest_inpt = m_inpt_hist.at(1);
        }
        // update m_sc to the time of the new oldest input (after inserting the new input)
        m_sc = predictTo(new_oldest_inpt.stamp);
        // insert the new input into the history buffer, causing the original oldest input to be removed
        m_inpt_hist.insert(next_inpt_it, inpt);
#ifdef REPREDICTOR_DEBUG
        if (m_inpt_hist.front() != new_oldest_inpt)
        {
          std::cerr << "Bad prediction of new oldest input!" << std::endl;
        }
#endif
      }
      else
      { // otherwise, just insert the new measurement
        m_inpt_hist.insert(next_inpt_it, inpt);
      }

#ifdef REPREDICTOR_DEBUG
      if (!checkMonotonicity(m_inpt_hist))
      {
        std::cerr << "Input history buffer is not monotonous after modification!" << std::endl;
      }
      std::cerr << "Added input (" << m_inpt_hist.size() << " inputs, " << m_meas_hist.size() << " measurements)" << std::endl;
#endif
    }
    //}

    /* addMeasurement() method //{ */
  /*!
    * \brief Adds one measurement to the measurement history buffer, removing the oldest element in the buffer if it is full.
    *
    * \param z      The measurement vector to be added.
    * \param R      The measurement noise covariance matrix, corresponding to the measurement vector.
    * \param stamp  Time stamp of the measurement vector and covariance matrix.
    * \param model  Optional pointer to a specific Model to be used with this measurement (eg. mapping it from different states). If it equals to nullptr, the default model specified in the constructor will be used.
    *
    * \note The measurement vector will not be added if it is older than the oldest one in the history buffer.
    *
    */
    void addMeasurement(const z_t& z, const R_t& R, const ros::Time& stamp, const ModelPtr& model = nullptr)
    {
      const meas_t meas {z, R, stamp, model};
      const auto next_meas_it = std::lower_bound(std::begin(m_meas_hist), std::end(m_meas_hist), meas, &Repredictor<Model>::earlier<meas_t>);
      m_meas_hist.insert(next_meas_it, meas);

#ifdef REPREDICTOR_DEBUG
      if (!checkMonotonicity(m_meas_hist))
      {
        std::cerr << "Measurement history buffer is not monotonous after modification!" << std::endl;
      }
      std::cerr << "Added measurement (" << m_inpt_hist.size() << " inputs, " << m_meas_hist.size() << " measurements)" << std::endl;
#endif
    }
    //}

  public:
    /* constructor //{ */
  /*!
    * \brief The main constructor.
    *
    * Initializes the Repredictor with the necessary initial and default values.
    *
    * \param x0             Initial state.
    * \param P0             Covariance matrix of the initial state uncertainty.
    * \param u0             Initial system input.
    * \param Q0             Default covariance matrix of the process noise.
    * \param t0             Time stamp of the initial state.
    * \param model          Default prediction and correction model.
    * \param inpt_hist_len  Length of the system input history buffer.
    * \param meas_hist_len  Length of the measurement history buffer.
    */
    Repredictor(const x_t& x0, const P_t& P0, const u_t& u0, const Q_t& Q0, const ros::Time& t0, const ModelPtr& model, const unsigned inpt_hist_len, const unsigned meas_hist_len)
      : m_sc{x0, P0}, m_default_model(model), m_inpt_hist(inpt_hist_t(inpt_hist_len)), m_meas_hist(meas_hist_t(meas_hist_len))
    {
      assert(inpt_hist_len > 0);
      assert(meas_hist_len > 0);
      addInput(u0, Q0, t0, model);
    };
    //}

  private:
    // state and covariance corresponding to the oldest input in the input history buffer
    statecov_t m_sc;
    // default model to use for updates
    ModelPtr m_default_model;

  private:
    /* helper structs and usings //{ */
    
    struct meas_t
    {
      z_t z;
      R_t R;
      ros::Time stamp;
      ModelPtr model;
    };
    
    struct inpt_t
    {
      u_t u;
      Q_t Q;
      ros::Time stamp;
      ModelPtr model;
    };
    
    using meas_hist_t = boost::circular_buffer<meas_t>;
    using inpt_hist_t = boost::circular_buffer<inpt_t>;
    
    //}

  private:
    // the history buffer for inputs
    inpt_hist_t m_inpt_hist;
    // the history buffer for measurements
    meas_hist_t m_meas_hist;

    // | ---------------- helper debugging methods ---------------- |
    /* checkMonotonicity() method //{ */
    template <typename T>
    bool checkMonotonicity(const T& buf)
    {
      if (buf.empty())
        return true;
      for (auto it = std::begin(buf)+1; it != std::end(buf); it++)
        if (earlier(*it, *(it-1)))
          return false;
      return true;
    }
    //}

    /* printBuffer() method //{ */
    template <typename T>
    void printBuffer(const T& buf)
    {
      if (buf.empty())
        return;
      const auto first_stamp = buf.front().stamp;
      std::cerr << "first stamp: " << first_stamp << std::endl;
      for (size_t it = 0; it < buf.size(); it++)
      {
        std::cerr << it << ":\t" << buf.at(it).stamp - first_stamp << std::endl;
      }
    }
    //}

  private:
    // | -------------- helper implementation methods ------------- |
    /* earlier() method //{ */
    template <typename T>
    static bool earlier(const T& ob1, const T& ob2)
    {
      return ob1.stamp < ob2.stamp;
    }
    //}

    /* predictFrom() method //{ */
    statecov_t predictFrom(const statecov_t& sc, const inpt_t& inpt, const ros::Time& from_stamp, const ros::Time& to_stamp)
    {
      const auto model = inpt.model == nullptr ? m_default_model : inpt.model;
      const auto dt = (to_stamp - from_stamp).toSec();
      return model->predict(sc, inpt.u, inpt.Q, dt);
    }
    //}

    /* correctFrom() method //{ */
    statecov_t correctFrom(const statecov_t& sc, const meas_t& meas)
    {
      const auto model = meas.model == nullptr ? m_default_model : meas.model;
      return model->correct(sc, meas.z, meas.R);
    }
    //}

  };
}  // namespace uav_localize


#endif // REPREDICTOR_H
