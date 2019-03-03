#ifndef REPREDICTOR_H
#define REPREDICTOR_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Time.h>
#include <functional>
#include "mrs_lib/Utils.h"

namespace mrs_lib
{
  template <int n_states, int n_inputs, int n_measurements, class Model>
  class Repredictor
  // TODO: initialization!
  {
  public:
    /* Helper classes and structs defines //{ */
    /* states, inputs etc. definitions (typedefs, constants etc) //{ */
    static const int n = n_states;
    static const int m = n_inputs;
    static const int p = n_measurements;

    using statecov_t = typename Model::statecov_t;
    using x_t = typename Model::x_t;  // state vector n*1
    using u_t = typename Model::u_t;  // input vector m*1
    using z_t = typename Model::z_t;  // measurement vector p*1

    using P_t = typename Model::P_t;  // state covariance n*n
    using R_t = typename Model::R_t;  // measurement covariance p*p
    //}

    /* info_t struct //{ */
    struct info_t
    {
      enum type_t
      {
        MEASUREMENT,
        INPUT,
        BOTH,
        NONE
      } type;
      z_t z;
      R_t R;
      u_t u;
      int param;
      ros::Time stamp;
    };
    //}

    /* statecov_stamped_t struct //{ */
    struct statecov_stamped_t
    {
      statecov_t statecov;
      ros::Time stamp;
    };
    //}

    /* Hist_point class definition //{ */
    // TODO: template
    class Hist_point
    {
      private:
        info_t info;
        statecov_t statecov;

      public:
        Hist_point(const info_t& info)
          : info(info)
        {};

        /* predict_to() method //{ */
        statecov_t predict_to(const ros::Time& stamp, const Model& model)
        {
          statecov_t ret;
          double dt = (stamp - info.stamp).toSec();
          switch (info.type)
          {
            case info_t::type_t::INPUT:
            case info_t::type_t::BOTH:
                ret = model.predict(statecov, info.u, dt, info.param);
                break;
            case info_t::type_t::MEASUREMENT:
            case info_t::type_t::NONE:
                ret = model.predict(statecov, u_t(), dt, info.param);
          }
          return ret;
        }
        //}

        /* update() method //{ */
        void update(const statecov_t& sc, const Model& model)
        {
          switch (info.type)
          {
            case info_t::type_t::MEASUREMENT:
            case info_t::type_t::BOTH:
              statecov = model.correct(sc, info.z, info.R, info.param);
              break;
            case info_t::type_t::INPUT:
            case info_t::type_t::NONE:
              statecov = sc;
              break;
          }
        }
        //}

        /* get_statecov_stamped() method //{ */
        statecov_stamped_t get_statecov_stamped() const
        {
          return {statecov, info.stamp};
        }
        //}

        /* get_statecov() method //{ */
        statecov_t get_statecov() const
        {
          return statecov;
        }
        //}

        /* get_stamp() method //{ */
        ros::Time get_stamp() const
        {
          return info.stamp;
        }
        //}

        /* get_info() method //{ */
        info_t get_info() const
        {
          return info;
        }
        //}
    };
    //}

    using hist_t = boost::circular_buffer<Hist_point>;
    //}

    /* predict_to() method //{ */
    statecov_t predict_to(const ros::Time& stamp)
    {
      return std::end(m_hist)->predict_to(stamp, m_model);
    }
    //}

    /* apply_new_measurement() method //{ */
    void apply_new_measurement(const z_t& z, const R_t& R, const ros::Time& stamp, int param = 0)
    {
      info_t info;
      info.type = info_t::type_t::MEASUREMENT;
      info.z = z;
      info.R = R;
      info.u = u_t();
      info.param = param;
      info.stamp = stamp;
      apply_new_info(info);
    }
    //}

    /* apply_new_input() method //{ */
    void apply_new_input(const u_t& u, const ros::Time& stamp, int param = 0)
    {
      info_t info;
      info.type = info_t::type_t::INPUT;
      info.z = z_t();
      info.R = R_t();
      info.u = u;
      info.param = param;
      info.stamp = stamp;
      apply_new_info(info);
    }
    //}

    /* apply_new_info() method //{ */
    void apply_new_info(info_t info)
    {
      // there must already be at least one history point in the buffer (which should be true)
      const typename hist_t::iterator histpt_prev_it = remove_const(find_prev(info.stamp, m_hist), m_hist);
      /* const typename hist_t::iterator histpt_prev_it = find_prev(info.stamp, m_hist); */
      const typename hist_t::iterator histpt_next_it = histpt_prev_it+1;
    
      // if this info is a measurement, copy the input value from previous history point info
      if (info.type == info_t::type_t::MEASUREMENT)
        info.u = histpt_prev_it->get_info().u;
      // create the new history point
      Hist_point histpt_n(info);
      // initialize it with states from the previous history point
      statecov_t sc = histpt_prev_it->predict_to(histpt_n.get_stamp(), m_model);
      histpt_n.update(sc, m_model);
      // insert the new history point into the history buffer (potentially kicking out the oldest
      // point at the beginning of the buffer)
      const typename hist_t::iterator histpt_new_it = m_hist.insert(histpt_next_it, histpt_n);
      // update the history according to the new history point
      update_history(histpt_new_it);
    }
    //}

    /* get_history_states() method //{ */
    std::vector<statecov_stamped_t> get_history_states()
    {
      std::vector<statecov_stamped_t> ret;
      ret.reserve(m_hist.size());
      for (const auto& hist_pt : m_hist)
        ret.push_back(hist_pt.get_statecov_stamped());
      return ret;
    }
    //}

  public:
    Repredictor(const Model& model, const x_t& x0, const P_t& P0, const ros::Time& t0, const unsigned hist_len)
      : m_model(model), m_hist(hist_t(hist_len))
    {
      assert(hist_len > 0);
      // Initialize the history with the first state and covariance
      info_t init_info;
      init_info.type = info_t::type_t::NONE;  // the first info is empty (no known input or measurement)
      init_info.stamp = t0;
      const statecov_t init_sc {x0, P0};
      Hist_point init_pt(init_info);
      init_pt.update(init_sc, m_model);
      m_hist.push_back(init_pt);
    };

  private:
    Model m_model;
    hist_t m_hist;

  private:
    /* find_prev() method and helpers //{ */
    template <class T>
    static void slice_in_half(const typename T::const_iterator b_in, const typename T::const_iterator e_in, typename T::const_iterator& b_out,
                       typename T::const_iterator& m_out, typename T::const_iterator& e_out)
    {
      b_out = b_in;
      m_out = b_in + (e_in - b_in) / 2;
      e_out = e_in;
    }

    template <class T>
    static const ros::Time get_stamp(const T& t)
    {
      return t.stamp;
    }

    static const ros::Time get_stamp(const Hist_point& t)
    {
      return t.get_stamp();
    }

    template <class T>
    static typename T::const_iterator find_prev(const ros::Time& stamp, const T& bfr)
    {
      using it_t = typename T::const_iterator;
      it_t b = std::begin(bfr);
      if (bfr.empty())
        return b;
      it_t e = std::end(bfr) - 1;
      it_t m = e;
      do
      {
        const double cmp = (get_stamp(*m) - stamp).toSec();
        if (cmp > 0.0)
          slice_in_half<T>(b, m, b, m, e);  // avoiding tuples for better performance
        else if (cmp < 0.0)
          slice_in_half<T>(m, e, b, m, e);
        else
          break;
      } while (b != m);
      return m;
    }
    //}

    /* update_history() method //{ */
    void update_history(const typename hist_t::iterator& start_it)
    {
      using it_t = typename hist_t::iterator;
      it_t cur_it = start_it;
      it_t next_it = start_it+1;

      while (next_it != m_hist.end())
      {
        statecov_t sc = cur_it->predict_to(next_it->get_stamp(), m_model);
        next_it->update(sc, m_model);

        cur_it++;
        next_it++;
      }
    }
    //}

  };
}  // namespace uav_localize


#endif // REPREDICTOR_H
