#ifndef REPREDICTOR_H
#define REPREDICTOR_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Time.h>
#include "mrs_lib/Utils.h"

namespace mrs_lib
{
  template <int n_states, int n_inputs, int n_measurements>
  class Repredictor
  {
  public:
    /* Helper classes and structs defines //{ */
    /* states, inputs etc. definitions (typedefs, constants etc) //{ */
    static const unsigned n = n_states;
    static const unsigned m = n_inputs;
    static const unsigned p = n_measurements;

    typedef Eigen::Matrix<double, n, 1> x_t;  // state vector n*1
    typedef Eigen::Matrix<double, m, 1> u_t;  // input vector m*1
    typedef Eigen::Matrix<double, p, 1> z_t;  // measurement vector p*1

    typedef Eigen::Matrix<double, n, n> P_t;  // state covariance n*n
    typedef Eigen::Matrix<double, p, p> R_t;  // measurement covariance p*p
    //}

    /* statecov_t struct //{ */
    struct statecov_t
    {
      x_t x;
      P_t P;
    };
    //}

    /* info_t struct //{ */
    struct info_t
    {
      enum type_t
      {
        MEASUREMENT,
        INPUT
      } type;
      z_t z;
      R_t R;
      u_t u;
      ros::Time stamp;
    };
    //}

    /* Model virtual class //{ */
    class Model
    {
      public:
        virtual statecov_t correct(const statecov_t& sc, const z_t& z, const R_t& R) const = 0;
        virtual statecov_t predict(const statecov_t& sc, const u_t& u, double dt) const = 0;
    };
    //}

    /* Hist_point class definition //{ */
    // TODO: template
    class Hist_point
    {
      private:
        info_t info;
        const Model* model;
        statecov_t statecov;

      public:
        Hist_point(const info_t& info, const Model* model)
          : info(info), model(model)
        {};

        /* predict_to() method //{ */
        statecov_t predict_to(const ros::Time& stamp)
        {
          double dt = (stamp - info.stamp).toSec();
          return model->predict(statecov, info.u, dt);
        }
        //}

        /* update() method //{ */
        void update(const statecov_t& sc)
        {
          if (info.type == info_t::type_t::MEASUREMENT)
            statecov = model->correct(sc, info.z, info.R);
          else
            statecov = sc;
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

    /* apply_new_measurement() method //{ */
    void apply_new_measurement(const z_t& z, const R_t& R, const ros::Time& stamp, const Model* model)
    {
      info_t info;
      info.type = info_t::type_t::MEASUREMENT;
      info.z = z;
      info.R = R;
      info.u = u_t();
      info.stamp = stamp;
      apply_new_info(info, model);
    }
    //}

    /* apply_new_input() method //{ */
    void apply_new_input(const u_t& u, const ros::Time& stamp, const Model* model)
    {
      info_t info;
      info.type = info_t::type_t::INPUT;
      info.z = z_t();
      info.R = R_t();
      info.u = u;
      info.stamp = stamp;
      apply_new_info(info, model);
    }
    //}

    /* apply_new_info() method //{ */
    void apply_new_info(info_t info, const Model* model)
    {
      // there must already be at least one history point in the buffer (which should be true)
      const typename hist_t::iterator histpt_prev_it = remove_const(find_prev(info.stamp, m_hist), m_hist);
      /* const typename hist_t::iterator histpt_prev_it = find_prev(info.stamp, m_hist); */
      const typename hist_t::iterator histpt_next_it = histpt_prev_it+1;
    
      // if this info is a measurement, copy the input value from previous history point info
      if (info.type == info_t::type_t::MEASUREMENT)
        info.u = histpt_prev_it->get_info().u;
      // create the new history point
      Hist_point histpt_n(info, model);
      // initialize it with states from the previous history point
      statecov_t sc = histpt_prev_it->predict_to(histpt_n.get_stamp());
      histpt_n.update(sc);
      // insert the new history point into the history buffer (potentially kicking out the oldest
      // point at the beginning of the buffer)
      const typename hist_t::iterator histpt_new_it = m_hist.insert(histpt_next_it, histpt_n);
      // update the history according to the new history point
      update_history(histpt_new_it);
    }
    //}

  public:
    Repredictor()
    {};

  private:
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
        statecov_t sc = cur_it->predict_to(next_it->get_stamp());
        next_it->update(sc);

        cur_it++;
        next_it++;
      }
    }
    //}

  };
}  // namespace uav_localize


#endif // REPREDICTOR_H
