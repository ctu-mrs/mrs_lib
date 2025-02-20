#ifndef REPREDICTOR_H
#define REPREDICTOR_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/utils.h>

namespace mrs_lib
{
  /**
   * \brief Implementation of the Repredictor for fusing measurements with variable delays.
   *
   * A standard state-space system model is assumed for the repredictor with system inputs and measurements,
   * generated from the system state vector. The inputs and measurements may be delayed with varying durations (an older measurement may become available after
   * a newer one). A typical use-case scenario is when you have one "fast" but imprecise sensor and one "slow" but precise sensor and you want to use them both
   * to get a good prediction (eg. height from the Garmin LiDAR, which comes at 100Hz, and position from a SLAM, which comes at 10Hz with a long delay). If the
   * slow sensor is significantly slower than the fast one, fusing its measurement at the time it arrives without taking into account the sensor's delay may
   * significantly bias your latest estimate.
   *
   * To accomodate this, the Repredictor keeps a buffer of N last inputs and measurements (N is specified
   * in the constructor). This buffer is then used to re-predict the desired state to a specific time, as
   * requested by the user. Note that the re-prediction is evaluated in a lazy manner only when the user requests it,
   * so it goes through the whole history buffer every time a prediction is requested.
   *
   * The Repredictor utilizes a fusion Model (specified as the template parameter), which should implement
   * the predict() and correct() methods. This Model is used for fusing the system inputs and measurements
   * as well as for predictions. Typically, this Model will be some kind of a Kalman Filter (LKF, UKF etc.).
   * \note The Model should be able to accomodate predictions with varying time steps in order for
   * the Repredictor to work correctly (see eg. the varstepLKF class).
   *
   * \tparam Model                  the prediction and correction model (eg. a Kalman Filter).
   * \tparam disable_reprediction   if true, reprediction is disabled and the class will act like a dumb LKF (for evaluation purposes).
   *
   */
  template <class Model, bool disable_reprediction = false>
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
    template <bool check = disable_reprediction>
    std::enable_if_t<!check, statecov_t> predictTo(const rclcpp::Time& to_stamp)
    {

      assert(!m_history.empty());
      auto hist_it = std::begin(m_history);

      if (hist_it->is_measurement)
      {
        // if the first history point is measurement, don't use it for correction (to prevent it from being used twice)
        hist_it->is_measurement = false;
      }
      // cur_stamp corresponds to the time point of cur_sc estimation
      auto cur_stamp = hist_it->stamp;
      // cur_sc is the current state and covariance estimate
      auto cur_sc = m_sc;
      do
      {
        cur_sc.stamp = hist_it->stamp;
        // do the correction if current history point is a measurement
        if ((hist_it->is_measurement))
          cur_sc = correctFrom(cur_sc, *hist_it);

        // decide whether to predict to the next history point or straight to the desired stamp already
        // (whichever comes sooner or directly to the desired stamp if no further history point is available)
        rclcpp::Time next_stamp = to_stamp;

        if ((hist_it + 1) != std::end(m_history) && (hist_it + 1)->stamp <= to_stamp)
          next_stamp = (hist_it + 1)->stamp;

        // do the prediction
        cur_sc = predictFrom(cur_sc, *hist_it, cur_stamp, next_stamp);
        cur_stamp = next_stamp;

        // increment the history pointer
        hist_it++;
      } while (hist_it != std::end(m_history) && hist_it->stamp <= to_stamp);

      cur_sc.stamp = to_stamp;

      return cur_sc;
    }

    /*!
     * \brief Estimates the system state and covariance matrix at the specified time.
     *
     * The measurement and system input histories are used to estimate the state vector and
     * covariance matrix values at the specified time, which are returned.
     *
     * \param to_stamp   The desired time at which the state vector and covariance matrix should be estimated.
     * \return           Returns the estimated state vector and covariance matrix in a single struct.
     *
     * \note This is the variant of the method when reprediction is disabled and will function like a dumb LKF.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<check, statecov_t> predictTo(const rclcpp::Time& to_stamp)
    {

      assert(!m_history.empty());
      const auto& info = m_history.front();
      auto sc = predictFrom(m_sc, info, info.stamp, to_stamp);
      sc.stamp = to_stamp;

      return sc;
    }
    //}

    /* addInputChangeWithNoise() method //{ */
    /*!
     * \brief Adds one system input to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param u      The system input vector to be added.
     * \param Q      The process noise covariance matrix.
     * \param stamp  Time stamp of the input vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this input (eg. mapping it to different states). If it equals to nullptr, the default
     * model specified in the constructor will be used.
     *
     * \note The system input vector will not be added if it is older than the oldest element in the history buffer.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<!check> addInputChangeWithNoise(const u_t& u, const Q_t& Q, const rclcpp::Time& stamp, const ModelPtr& model = nullptr)
    {
      const info_t info(stamp, u, Q, model);
      // find the next point in the history buffer
      const auto next_it = std::lower_bound(std::begin(m_history), std::end(m_history), info, &Repredictor<Model>::earlier);
      // add the point to the history buffer
      const auto added = addInfo(info, next_it);
      // update all measurements following the newly added system input up to the next system input
      if (added != std::end(m_history))
        for (auto it = added + 1; it != std::end(m_history) && it->is_measurement; it++)
          it->updateUsing(info);
    }

    /*!
     * \brief Adds one system input to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param u      The system input vector to be added.
     * \param Q      The process noise covariance matrix.
     * \param stamp  Time stamp of the input vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this input (eg. mapping it to different states). If it equals to nullptr, the default
     * model specified in the constructor will be used.
     *
     * \note This is the variant of the method when reprediction is disabled and will function like a dumb LKF.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<check> addInputChangeWithNoise(const u_t& u, const Q_t& Q, [[maybe_unused]] const rclcpp::Time& stamp, const ModelPtr& model = nullptr)
    {
      if (m_history.empty())
        m_history.push_back({stamp});
      m_history.front().u = u;
      m_history.front().Q = Q;
      m_history.front().stamp = stamp;
      m_history.front().predict_model = model;
    }
    //}

    /* addInputChange() method //{ */
    /*!
     * \brief Adds one system input to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param u      The system input vector to be added.
     * \param stamp  Time stamp of the input vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this input (eg. mapping it to different states). If it equals to nullptr, the default
     * model specified in the constructor will be used.
     *
     * \note The system input vector will not be added if it is older than the oldest element in the history buffer.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<!check> addInputChange(const u_t& u, const rclcpp::Time& stamp, const ModelPtr& model = nullptr)
    {

      assert(!m_history.empty());

      // find the next point in the history buffer
      const auto next_it = std::lower_bound(std::begin(m_history), std::end(m_history), stamp, &Repredictor<Model>::earlier);

      // get the previous history point (or the first one to avoid out of bounds)
      const auto prev_it = next_it == std::begin(m_history) ? next_it : next_it - 1;

      // initialize a new history info point
      const info_t info(stamp, u, prev_it->Q, model);

      // add the point to the history buffer
      const auto added = addInfo(info, next_it);

      // update all measurements following the newly added system input up to the next system input
      if (added != std::end(m_history))
        for (auto it = added + 1; it != std::end(m_history) && it->is_measurement; it++)
          it->updateUsing(info);
    }

    /*!
     * \brief Adds one system input to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param u      The system input vector to be added.
     * \param stamp  Time stamp of the input vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this input (eg. mapping it to different states). If it equals to nullptr, the default
     * model specified in the constructor will be used.
     *
     * \note This is the variant of the method when reprediction is disabled and will function like a dumb LKF.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<check> addInputChange(const u_t& u, [[maybe_unused]] const rclcpp::Time& stamp, const ModelPtr& model = nullptr)
    {
      if (m_history.empty())
        m_history.push_back({stamp});
      m_history.front().u = u;
      m_history.front().stamp = stamp;
      m_history.front().predict_model = model;
    }
    //}

    /* addProcessNoiseChange() method //{ */
    /*!
     * \brief Adds one system input to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param Q      The process noise covariance matrix.
     * \param stamp  Time stamp of the input vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this covariance matrix (eg. mapping it to different states). If it equals to nullptr,
     * the default model specified in the constructor will be used.
     *
     * \note The new element will not be added if it is older than the oldest element in the history buffer.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<!check> addProcessNoiseChange(const Q_t& Q, const rclcpp::Time& stamp, const ModelPtr& model = nullptr)
    {

      assert(!m_history.empty());

      // find the next point in the history buffer
      const auto next_it = std::lower_bound(std::begin(m_history), std::end(m_history), stamp, &Repredictor<Model>::earlier);

      // get the previous history point (or the first one to avoid out of bounds)
      const auto prev_it = next_it == std::begin(m_history) ? next_it : next_it - 1;

      // initialize a new history info point
      const info_t info(stamp, prev_it->u, Q, model);

      // add the point to the history buffer
      const auto added = addInfo(info, next_it);

      // update all measurements following the newly added system input up to the next system input
      if (added != std::end(m_history))
        for (auto it = added + 1; it != std::end(m_history) && it->is_measurement; it++)
          it->updateUsing(info);
    }

    /*!
     * \brief Adds one system input to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param Q      The process noise covariance matrix.
     * \param stamp  Time stamp of the input vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this covariance matrix (eg. mapping it to different states). If it equals to nullptr,
     * the default model specified in the constructor will be used.
     *
     * \note This is the variant of the method when reprediction is disabled and will function like a dumb LKF.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<check> addProcessNoiseChange(const Q_t& Q, [[maybe_unused]] const rclcpp::Time& stamp, const ModelPtr& model = nullptr)
    {

      if (m_history.empty())
        m_history.push_back({stamp});

      m_history.front().Q = Q;
      m_history.front().stamp = stamp;
      m_history.front().predict_model = model;
    }
    //}

    /* addMeasurement() method //{ */
    /*!
     * \brief Adds one measurement to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param z      The measurement vector to be added.
     * \param R      The measurement noise covariance matrix, corresponding to the measurement vector.
     * \param stamp  Time stamp of the measurement vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this measurement (eg. mapping it from different states). If it equals to nullptr, the
     * default model specified in the constructor will be used.
     *
     * \note The measurement vector will not be added if it is older than the oldest element in the history buffer.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<!check> addMeasurement(const z_t& z, const R_t& R, const rclcpp::Time& stamp, const ModelPtr& model = nullptr, const double& meas_id = -1)
    {

      assert(!m_history.empty());

      // helper variable for searching of the next point in the history buffer
      const auto next_it = std::lower_bound(std::begin(m_history), std::end(m_history), stamp, &Repredictor<Model>::earlier);

      // get the previous history point (or the first one to avoid out of bounds)
      const auto prev_it = next_it == std::begin(m_history) ? next_it : next_it - 1;

      // initialize a new history info point
      const info_t info(stamp, z, R, model, *prev_it, meas_id);

      // add the point to the history buffer
      addInfo(info, next_it);
    }

    /*!
     * \brief Adds one measurement to the history buffer, removing the oldest element in the buffer if it is full.
     *
     * \param z      The measurement vector to be added.
     * \param R      The measurement noise covariance matrix, corresponding to the measurement vector.
     * \param stamp  Time stamp of the measurement vector and covariance matrix.
     * \param model  Optional pointer to a specific Model to be used with this measurement (eg. mapping it from different states). If it equals to nullptr, the
     * default model specified in the constructor will be used.
     *
     * \note This is the variant of the method when reprediction is disabled and will function like a dumb LKF.
     *
     */
    template <bool check = disable_reprediction>
    std::enable_if_t<check> addMeasurement(const z_t& z, const R_t& R, const rclcpp::Time& stamp, const ModelPtr& model = nullptr, const double& meas_id = -1)
    {

      if (m_history.empty())
        m_history.push_back({stamp});

      auto& info = m_history.front();
      const rclcpp::Time to_stamp = stamp > info.stamp ? stamp : info.stamp;
      const auto sc = predictTo(to_stamp);

      info.z = z;
      info.R = R;
      info.stamp = to_stamp;
      info.is_measurement = true;
      info.meas_id = meas_id;
      info.correct_model = model;
      m_sc = correctFrom(sc, info);
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
     * \param hist_len       Length of the history buffer for system inputs and measurements.
     */
    Repredictor(const x_t& x0, const P_t& P0, const u_t& u0, const Q_t& Q0, const rclcpp::Time& t0, const ModelPtr& model, const unsigned hist_len)
        : m_sc{x0, P0}, m_default_model(model), m_history(history_t(hist_len))
    {

      assert(hist_len > 0);

      addInputChangeWithNoise(u0, Q0, t0, model);
    };

    /*!
     * \brief Empty constructor.
     *
     */
    Repredictor(){};

    /*!
     * \brief Variation of the constructor for cases without a system input.
     *
     * Initializes the Repredictor with the necessary initial and default values.
     * Assumes the system input is zero at t0.
     *
     * \param x0             Initial state.
     * \param P0             Covariance matrix of the initial state uncertainty.
     * \param Q0             Default covariance matrix of the process noise.
     * \param t0             Time stamp of the initial state.
     * \param model          Default prediction and correction model.
     * \param hist_len       Length of the history buffer for system inputs and measurements.
     */
    Repredictor(const x_t& x0, const P_t& P0, const Q_t& Q0, const rclcpp::Time& t0, const ModelPtr& model, const unsigned hist_len)
        : m_sc{x0, P0}, m_default_model(model), m_history(history_t(hist_len))
    {

      assert(hist_len > 0);

      const u_t u0{0};

      addInputChangeWithNoise(u0, Q0, t0, model);
    };
    //}

  protected:
    // state and covariance corresponding to the oldest element in the history buffer
    statecov_t m_sc;
    // default model to use for updates
    ModelPtr m_default_model;

  private:
    /* helper structs and usings //{ */

    struct info_t
    {
      rclcpp::Time stamp;

      // system input-related information
      u_t u;
      Q_t Q;
      ModelPtr predict_model;

      // measurement-related information (unused in case is_measurement=false)
      z_t z;
      R_t R;
      ModelPtr correct_model;
      bool is_measurement;
      int meas_id;

      // constructor for a dummy info (for searching in the history)
      info_t(const rclcpp::Time& stamp) : stamp(stamp), is_measurement(false){};

      // constructor for a system input
      info_t(const rclcpp::Time& stamp, const u_t& u, const Q_t& Q, const ModelPtr& model)
          : stamp(stamp), u(u), Q(Q), predict_model(model), is_measurement(false){};

      // constructor for a measurement
      info_t(const rclcpp::Time& stamp, const z_t& z, const R_t& R, const ModelPtr& model, const info_t& prev_info, const int& meas_id)
          : stamp(stamp), z(z), R(R), correct_model(model), is_measurement(true), meas_id(meas_id)
      {
        updateUsing(prev_info);
      };

      // copy system input-related information from a previous info
      void updateUsing(const info_t& info)
      {
        u = info.u;
        Q = info.Q;
        predict_model = info.predict_model;
      };
    };


    //}

  protected:
    using history_t = boost::circular_buffer<info_t>;
    // the history buffer
    history_t m_history;

    // | ---------------- helper debugging methods ---------------- |
    /* checkMonotonicity() method //{ */
    template <typename T>
    bool checkMonotonicity(const T& buf)
    {
      if (buf.empty())
        return true;
      for (auto it = std::begin(buf) + 1; it != std::end(buf); it++)
        if (earlier(*it, *(it - 1)))
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

    /* addInfo() method //{ */
    typename history_t::iterator addInfo(const info_t& info, const typename history_t::iterator& next_it)
    {
      // check if the new element would be added before the first element of the history buffer and ignore it if so
      if (next_it == std::begin(m_history) && !m_history.empty())
      {
        std::cout << "[Repredictor]: Added history point is older than the oldest by " << (next_it->stamp - info.stamp).seconds()
                  << "s. Ignoring it! Consider increasing the history buffer size (currently: " << m_history.size() << ")" << std::endl;
        return std::end(m_history);
      }

      // check if adding a new element would throw out the oldest one
      if (m_history.size() == m_history.capacity())
      {  // if so, first update m_sc
        // figure out, what will be the oldest element in the buffer after inserting the newly received one
        auto new_oldest = m_history.front();
        if (m_history.size() == 1 || (m_history.size() > 1 && info.stamp > m_history.at(0).stamp && info.stamp < m_history.at(1).stamp))
        {
          new_oldest = info;
        } else if (m_history.size() > 1)
        {
          new_oldest = m_history.at(1);
        }
        // update m_sc to the time of the new oldest element (after inserting the new element)
        m_sc = predictTo(new_oldest.stamp);
        // insert the new element into the history buffer, causing the original oldest element to be removed
      }

      // add the new point finally
      const auto ret = m_history.insert(next_it, info);
      /* debug check //{ */

#ifdef REPREDICTOR_DEBUG
      if (!checkMonotonicity(m_history))
      {
        std::cerr << "History buffer is not monotonous after modification!" << std::endl;
      }
      std::cerr << "Added info (" << m_history.size() << " total)" << std::endl;
#endif

      //}
      return ret;
    }
    //}

    /* earlier() method //{ */
    static bool earlier(const info_t& ob1, const info_t& ob2)
    {
      return ob1.stamp < ob2.stamp;
    }
    //}

    /* predictFrom() method //{ */
    statecov_t predictFrom(const statecov_t& sc, const info_t& inpt, const rclcpp::Time& from_stamp, const rclcpp::Time& to_stamp)
    {
      const auto model = inpt.predict_model == nullptr ? m_default_model : inpt.predict_model;
      const auto dt = (to_stamp - from_stamp).seconds();
      return model->predict(sc, inpt.u, inpt.Q, dt);
    }
    //}

    /* correctFrom() method //{ */
    statecov_t correctFrom(const statecov_t& sc, const info_t& meas)
    {
      assert(meas.is_measurement);
      const auto model = meas.correct_model == nullptr ? m_default_model : meas.correct_model;
      auto sc_tmp = sc;
      return model->correct(sc_tmp, meas.z, meas.R);
    }
    //}
  };
}  // namespace mrs_lib


#endif  // REPREDICTOR_H
