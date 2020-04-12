#ifndef REPREDICTOR_H
#define REPREDICTOR_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Time.h>
#include <functional>
#include <mrs_lib/utils.h>

namespace mrs_lib
{
  template <int n_states, int n_inputs, int n_measurements, class Model>
  class Repredictor
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
    using Q_t = typename Model::Q_t;  // process noise covariance n*n
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
      Q_t Q;
      u_t u;
      int param;
      ros::Time stamp;
    };
    //}

    /* stamped template structs //{ */
    template <typename T>
    struct stamped
    {
      T raw;
      ros::Time stamp;
    };
    using statecov_stamped_t = stamped<statecov_t>;
    using z_stamped_t = stamped<z_t>;
    using u_stamped_t = stamped<u_t>;
    //}

    /* Hist_point class definition //{ */
    // TODO: template
    class Hist_point
    {
      private:
        info_t m_info;
        statecov_t m_statecov;

      public:
        Hist_point(const info_t& info)
          : m_info(info)
        {};

        /* predict_to() method //{ */
        /* statecov_t predict_to(const ros::Time& stamp, const Model& model, u_t& last_u) */
        statecov_t predict_to(const ros::Time& stamp, const Model& model)
        {
          if (stamp == m_info.stamp)
            return m_statecov;
          if (stamp < m_info.stamp)
            std::cout << "blebleble-------------------------------------------------------------------------------------------------" << std::endl;
          statecov_t ret;
          double dt = (stamp - m_info.stamp).toSec();
          switch (get_type())
          {
            case info_t::type_t::INPUT:
            case info_t::type_t::BOTH:
#ifdef DEBUG
                std::cout << std::fixed << std::setprecision(2) <<  "Predicting using input from " << m_info.stamp.toSec() << " to " << stamp.toSec() << std::endl;
#endif
                ret = model.predict(m_statecov, m_info.u, m_info.Q, dt, m_info.param);
                /* last_u = m_info.u; */
                break;
            case info_t::type_t::MEASUREMENT:
            case info_t::type_t::NONE:
#ifdef DEBUG
                std::cout << std::fixed << std::setprecision(2) << "Predicting using no input from " << m_info.stamp.toSec() << " to " << stamp.toSec() << std::endl;
#endif
                /* ret = model.predict(m_statecov, last_u, dt, m_info.param); */
                ret = model.predict(m_statecov, u_t::Zero(), m_info.Q, dt, m_info.param);
          }
          return ret;
        }
        //}

        /* update() method //{ */
        void update(const statecov_t& sc, const Model& model)
        {
          switch (m_info.type)
          {
            case info_t::type_t::MEASUREMENT:
            case info_t::type_t::BOTH:
#ifdef DEBUG
                std::cout << std::fixed << std::setprecision(2) <<  "Applying correction at " << m_info.stamp.toSec() << std::endl;
#endif
                m_statecov = model.correct(sc, m_info.z, m_info.R, m_info.param);
                break;
            case info_t::type_t::INPUT:
            case info_t::type_t::NONE:
#ifdef DEBUG
                std::cout << std::fixed << std::setprecision(2) <<  "Ignoring correction at " << m_info.stamp.toSec() << std::endl;
#endif
                m_statecov = sc;
                break;
          }
        }
        //}

        /* get_statecov_stamped() method //{ */
        statecov_stamped_t get_statecov_stamped() const
        {
          return {m_statecov, m_info.stamp};
        }
        //}

        /* get_z_stamped() method //{ */
        z_stamped_t get_z_stamped() const
        {
          return {m_info.z, m_info.stamp};
        }
        //}

        /* get_u_stamped() method //{ */
        u_stamped_t get_u_stamped() const
        {
          return {m_info.u, m_info.stamp};
        }
        //}

        /* get_statecov() method //{ */
        statecov_t get_statecov() const
        {
          return m_statecov;
        }
        //}

        /* get_stamp() method //{ */
        ros::Time get_stamp() const
        {
          return m_info.stamp;
        }
        //}

        /* get_info() method //{ */
        info_t get_info() const
        {
          return m_info;
        }
        //}

        /* get_type() method //{ */
        typename info_t::type_t get_type() const
        {
          return m_info.type;
        }
        //}

        /* add_input() method //{ */
        void add_input(const u_t& new_input)
        {
          assert(get_type() == info_t::type_t::MEASUREMENT || get_type() == info_t::type_t::NONE);
          m_info.u = new_input;
          if (m_info.type == info_t::type_t::MEASUREMENT)
            m_info.type = info_t::type_t::BOTH;
          else
            m_info.type = info_t::type_t::INPUT;
        }
        //}
    };
    //}

    using hist_t = boost::circular_buffer<Hist_point>;
    using u_hist_t = boost::circular_buffer<Hist_point>;
    //}

    /* predict_to() method //{ */
    statecov_t predict_to(const ros::Time& stamp)
    {
      return m_hist.back().predict_to(stamp, m_model);
    }
    //}

    /* apply_new_measurement() method //{ */
    void apply_new_measurement(const z_t& z, const R_t& R, const ros::Time& stamp, int param = 0)
    {
#ifdef DEBUG
      std::cout << "Applying new measurement ---------------------------------------------------------" << std::endl;
#endif
      info_t info;
      info.type = info_t::type_t::MEASUREMENT;
      info.z = z;
      info.R = R;
      info.Q = Q_t();
      info.u = u_t();
      info.param = param;
      info.stamp = stamp;
      apply_new_info(info);
    }
    //}

    /* apply_new_input() method //{ */
    void apply_new_input(const u_t& u, const ros::Time& stamp, int param = 0)
    {
#ifdef DEBUG
      std::cout << "Applying new input       ---------------------------------------------------------" << std::endl;
#endif
      info_t info;
      info.type = info_t::type_t::INPUT;
      info.z = z_t();
      info.R = R_t();
      info.Q = Q_t();
      info.u = u;
      info.param = param;
      info.stamp = stamp;
      apply_new_info(info);
    }
    //}

    /* apply_new_info() method //{ */
    void apply_new_info(info_t info)
    {
      // ignore any info, which is older than oldest element in the buffer
      if (info.stamp < m_hist.front().get_stamp())
        return;
      // there must already be at least one history point in the buffer (which should be true)
      const typename hist_t::iterator histpt_prev_it = remove_const(find_prev(info.stamp, m_hist), m_hist);
      /* const typename hist_t::iterator histpt_prev_it = find_prev(info.stamp, m_hist); */
      const typename hist_t::iterator histpt_next_it = histpt_prev_it+1;
    
      // create the new history point
      Hist_point histpt_n(info);
      // initialize it with states from the previous history point
      statecov_t sc = histpt_prev_it->predict_to(histpt_n.get_stamp(), m_model);
      histpt_n.update(sc, m_model);
#ifdef DEBUG
      print_debug_insertion(histpt_prev_it, histpt_n);
#endif
      // insert the new history point into the history buffer (potentially kicking out the oldest
      // point at the beginning of the buffer)
      const typename hist_t::iterator histpt_new_it = m_hist.insert(histpt_next_it, histpt_n);
#ifdef DEBUG
      print_debug_types();
#endif
      if (info.type == info_t::type_t::INPUT || info.type == info_t::type_t::BOTH)
      {
        const typename u_hist_t::iterator u_prev_it = remove_const(find_prev(info.stamp, m_u_hist), m_u_hist);
        // insert the new input history point to the buffer of inputs
        m_u_hist.insert(u_prev_it+1, histpt_n);
        // update all measurements after this input which don't have an input
        update_history_new_input(histpt_new_it+1, info.u);
      }
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

    /* get_history_measurements() method //{ */
    std::vector<z_stamped_t> get_history_measurements()
    {
      std::vector<z_stamped_t> ret;
      ret.reserve(m_hist.size());
      for (const auto& hist_pt : m_hist)
        if (hist_pt.get_type() == info_t::type_t::MEASUREMENT || hist_pt.get_type() == info_t::type_t::BOTH)
          ret.push_back(hist_pt.get_z_stamped());
      return ret;
    }
    //}

    /* get_history_inputs() method //{ */
    std::vector<u_stamped_t> get_history_inputs()
    {
      std::vector<u_stamped_t> ret;
      ret.reserve(m_hist.size());
      for (const auto& hist_pt : m_hist)
        if (hist_pt.get_type() == info_t::type_t::INPUT)
          ret.push_back(hist_pt.get_u_stamped());
      return ret;
    }
    //}

  public:
    Repredictor(const Model& model, const x_t& x0, const P_t& P0, const ros::Time& t0, const unsigned hist_len)
      : m_model(model), m_hist(hist_t(hist_len)), m_u_hist(u_hist_t(hist_len))
    {
      assert(hist_len > 0);
      // Initialize the history with the first state and covariance
      info_t init_info;
      init_info.type = info_t::type_t::NONE;  // the first info is empty (no known input or measurement)
      init_info.z = z_t();
      init_info.R = R_t();
      init_info.Q = Q_t();
      init_info.u = u_t::Zero();
      init_info.param = 0;
      init_info.stamp = t0;
      const statecov_t init_sc {x0, P0};
      Hist_point init_pt(init_info);
      init_pt.update(init_sc, m_model);
      m_hist.push_back(init_pt);
      m_u_hist.push_back(init_pt);
    };

  private:
    Model m_model;
    hist_t m_hist;
    u_hist_t m_u_hist;

  private:
    /* debug printing methods //{ */
    void print_debug_insertion(const typename hist_t::const_iterator& histpt_prev_it, const Hist_point& histpt_n)
    {
      for (typename hist_t::const_iterator it = std::begin(m_hist); it != std::end(m_hist); it++)
      {
        std::cout << std::fixed << std::setprecision(1) << it->get_stamp().toSec();
        if (it == histpt_prev_it)
          std::cout << std::fixed << std::setprecision(1) << " --\\" << histpt_n.get_stamp().toSec() << "/";
        else
          std::cout << " ";
        if (it != std::end(m_hist)-1)
          std::cout << "-- ";
      }
      std::cout << std::endl;
    }
    void print_debug_types()
    {
      for (typename hist_t::const_iterator it = std::begin(m_hist); it != std::end(m_hist); it++)
      {
        switch (it->get_type())
        {
          case info_t::type_t::NONE:
              std::cout << "NONE   ";
              break;
          case info_t::type_t::INPUT:
              std::cout << "INPU   ";
              break;
          case info_t::type_t::MEASUREMENT:
              std::cout << "MEAS   ";
              break;
          case info_t::type_t::BOTH:
              std::cout << "BOTH   ";
              break;
        }
      }
      std::cout << std::endl;
    }
    //}
    
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
#ifdef DEBUG
      for (typename hist_t::const_iterator it = std::begin(m_hist); it != start_it; it++)
        std::cout << "---    ";
      for (typename hist_t::const_iterator it = start_it; it != std::end(m_hist); it++)
        std::cout << "^^^    ";
      std::cout << std::endl;
#endif

      while (next_it != m_hist.end())
      {
        statecov_t sc = cur_it->predict_to(next_it->get_stamp(), m_model);
        next_it->update(sc, m_model);
#ifdef DEBUG
        if (!sc.x.allFinite())
          std::cout << "State contains NaNs!" << std::endl;
        if (!sc.P.allFinite())
          std::cout << "Covariance contains NaNs!" << std::endl;
#endif

        cur_it++;
        next_it++;
      }
    }
    //}

    /* update_history_new_input() method //{ */
    void update_history_new_input(const typename hist_t::iterator& start_it, const u_t& new_input)
    {
      using it_t = typename hist_t::iterator;
      it_t cur_it = start_it;
      while (cur_it != m_hist.end()
         && (cur_it->get_type() == info_t::type_t::MEASUREMENT || cur_it->get_type() == info_t::type_t::NONE))
      {
        cur_it->add_input(new_input);
        cur_it++;
      }
    }
    //}

  };
}  // namespace uav_localize


#endif // REPREDICTOR_H
