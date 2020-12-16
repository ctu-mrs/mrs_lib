#ifndef REPREDICTOR_H
#define REPREDICTOR_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Time.h>
#include <functional>
#include <mrs_lib/utils.h>

namespace mrs_lib
{
  template <class Model>
  class Repredictor
  {
  public:
    /* Helper classes and structs defines //{ */
    /* states, inputs etc. definitions (typedefs, constants etc) //{ */
    static const int n = Model::n;
    static const int m = Model::m;
    static const int p = Model::p;

    using ModelPtr = typename std::shared_ptr<Model>;
    using statecov_t = typename Model::statecov_t;
    using x_t = typename Model::x_t;  // state vector n*1
    using u_t = typename Model::u_t;  // input vector m*1
    using z_t = typename Model::z_t;  // measurement vector p*1

    using P_t = typename Model::P_t;  // state covariance n*n
    using R_t = typename Model::R_t;  // measurement covariance p*p
    using Q_t = typename Model::Q_t;  // process noise covariance n*n
    //}

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

    /* predictTo() method //{ */
    statecov_t predictTo(const ros::Time& stamp)
    {
      auto inpt_it = std::begin(m_inpt_hist);
      auto meas_it = std::begin(m_meas_hist);
      auto cur_stamp = inpt_it->stamp;
      auto cur_sc = m_sc;
      while (!(inpt_it == std::end(m_inpt_hist) && meas_it == std::end(m_meas_hist)) && cur_stamp < stamp)
      {
        ros::Time next_inpt_stamp = stamp;
        if ((inpt_it+1) != std::end(m_inpt_hist))
          next_inpt_stamp = (inpt_it+1)->stamp;

        ros::Time meas_stamp = stamp;
        if (meas_it != std::end(m_meas_hist))
          meas_stamp = meas_it->stamp;

        // decide whether we should predict to the next input or to the current measurement
        // (whichever is sooner according to the stamp)
        const bool use_meas_next = meas_stamp < next_inpt_stamp;
        const auto to_stamp = use_meas_next ? meas_stamp : next_inpt_stamp;

        // do the prediction
        cur_sc = predictFrom(cur_sc, *inpt_it, cur_stamp, to_stamp);
        if (use_meas_next)
          cur_sc = correctFrom(cur_sc, *meas_it);
        cur_stamp = to_stamp;

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
      cur_sc = predictFrom(cur_sc, *inpt_it, cur_stamp, stamp);
      return cur_sc;
    }
    //}

    /* addMeasurement() method //{ */
    void addMeasurement(const z_t& z, const R_t& R, const ros::Time& stamp, const ModelPtr& model = nullptr)
    {
      const meas_t meas {z, R, stamp, model};
      const auto next_meas_it = std::lower_bound(std::begin(m_meas_hist), std::end(m_meas_hist), meas, &Repredictor<Model>::earlier<meas_t>);
      m_meas_hist.insert(next_meas_it, meas);

      if (!checkMonotonicity(m_meas_hist))
      {
        std::cerr << "Measurement history buffer is not monotonous after modification!" << std::endl;
      }
    }
    //}

    /* addInput() method //{ */
    void addInput(const u_t& u, const Q_t& Q, const ros::Time& stamp, const ModelPtr& model = nullptr)
    {
      const inpt_t inpt {u, Q, stamp, model};
      const auto next_inpt_it = std::lower_bound(std::begin(m_inpt_hist), std::end(m_inpt_hist), inpt, &Repredictor<Model>::earlier<inpt_t>);
      // check if adding a new input would throw out the oldest one
      if (m_inpt_hist.size() == m_inpt_hist.capacity())
      { // if so, first update m_sc
        // remember the original oldest input
        const auto orig_oldest_inpt = m_inpt_hist.begin();
        // insert the new input into the history buffer, causing the original oldest input to be removed
        m_inpt_hist.insert(next_inpt_it, inpt);
        // get the current oldest input after the insertion of the new information
        const auto curr_oldest_inpt = m_inpt_hist.begin();
        // update m_sc
        // TODO: Take into account measurements as well!
        m_sc = predictFrom(m_sc, *orig_oldest_inpt, orig_oldest_inpt->stamp, curr_oldest_inpt->stamp);
      }
      else
      { // otherwise, just insert the new measurement
        m_inpt_hist.insert(next_inpt_it, inpt);
      }

      if (!checkMonotonicity(m_inpt_hist))
      {
        std::cerr << "Input history buffer is not monotonous after modification!" << std::endl;
      }
    }
    //}

  public:
    Repredictor(const x_t& x0, const P_t& P0, const u_t& u0, const Q_t& Q0, const ros::Time& t0, const ModelPtr& model, const unsigned hist_len)
      : m_sc{x0, P0}, m_default_model(model), m_meas_hist(meas_hist_t(hist_len)), m_inpt_hist(inpt_hist_t(hist_len))
    {
      assert(hist_len > 0);
      addInput(u0, Q0, t0, model);
    };

  private:
    // state and covariance corresponding to the oldest input in the input history buffer
    statecov_t m_sc;
    // default model to use for updates
    ModelPtr m_default_model;

  private:
    // the history buffer for measurements
    meas_hist_t m_meas_hist;
    // the history buffer for inputs
    inpt_hist_t m_inpt_hist;
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

  private:
    template <typename T>
    static bool earlier(const T& ob1, const T& ob2)
    {
      return ob1.stamp < ob2.stamp;
    }

    statecov_t predictFrom(const statecov_t& sc, const inpt_t& inpt, const ros::Time& from_stamp, const ros::Time& to_stamp)
    {
      const auto model = inpt.model == nullptr ? m_default_model : inpt.model;
      const auto dt = (to_stamp - from_stamp).toSec();
      return model->predict(sc, inpt.u, inpt.Q, dt);
    }

    statecov_t correctFrom(const statecov_t& sc, const meas_t& meas)
    {
      const auto model = meas.model == nullptr ? m_default_model : meas.model;
      return model->correct(sc, meas.z, meas.R);
    }

  };
}  // namespace uav_localize


#endif // REPREDICTOR_H
