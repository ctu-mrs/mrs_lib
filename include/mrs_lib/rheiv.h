// clang: MatousFormat
/**  \file
     \brief Defines RHEIV and related stuff for surface fitting to points with known covariances according to \cite FNSandHEIV.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef HEIV_H
#define HEIV_H

#include <Eigen/Dense>
#include <iostream>
#include <chrono>

namespace mrs_lib
{
  /* eigenvector_exception //{ */
  
  /*!
    * \brief This exception may be thrown when solving the generalized eigenvalue problem with the M and N matrices.
    *
    * You should catch this exception in your code and act accordingly if it appears
    * (e.g. ask for the last valid parameter estimate).
    */
    struct eigenvector_exception : public std::exception
    {
    /*!
      * \brief Returns the error message, describing what caused the exception.
      *
      * \return  The error message, describing what caused the exception.
      */
      virtual const char* what() const throw() override
      {
        return "Could not compute matrix eigenvectors.";
      }
    };
  
  //}

  /* class RHEIV //{ */
  /**
  * \brief Implementation of the Reduced Heteroscedastic Errors In Variables surface fitting algorithm \cite FNSandHEIV.
  *
  * This class estimates a vector of parameters \f$ \mathbf{\theta} \f$ of a model, which is defined by an equation
  * \f$ \mathbf{\theta}^T \mathbf{u}\left( \mathbf{x} \right) = 0 \f$,
  * where \f$ \mathbf{x} \f$ is a data sample vector and \f$ \mathbf{u}\left( \mathbf{x} \right) \f$ is a vector,
  * obtained by transforming \f$ \mathbf{x} \f$ in a problem-dependent manner.
  *
  * Note that \f$ \mathbf{u}\left( \mathbf{x} \right) \f$ must fulfill two conditions:
  * - each element of \f$ \mathbf{u}\left( \mathbf{x} \right) \f$ is a quadratic form of a vector \f$ \left[ \mathbf{x}, 1 \right] \f$, and
  * - the last element of \f$ \mathbf{u}\left( \mathbf{x} \right) \f$ is equal to one.
  *
  * Such model can be used to describe e.g. various surfaces, such as planes or conics, which makes this class a useful tool for computer vision.
  * The model is described in the context of the RHEIV class by the length of the vector \f$ \mathbf{x} \f$, the length of the vector \f$ \mathbf{\theta} \f$,
  * and the Jacobian matrix of \f$ \partial_{\mathbf{x}} \mathbf{z}\left( \mathbf{x} \right) \f$, where \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ is defined
  * by the formula \f$ \mathbf{u}\left( \mathbf{x} \right) = \left[ \mathbf{z}\left( \mathbf{x} \right), 1 \right] \f$.
  *
  * For more information, see \cite FNSandHEIV.
  *
  */
  template <int n_states, int n_params>
  class RHEIV
  {
    static_assert(n_states <= n_params, "Sorry, n_states must be smaller or equal to n_params");
  public:
    /* RHEIV definitions (typedefs, constants etc) //{ */

    static const int k = n_states;            /*!< \brief Length of the state vector \p x */
    static const int l = n_params;            /*!< \brief Length of the parameter vector \f$ \mathbf{\theta} \f$ */
    static const int lr = l - 1;              /*!< \brief Length of the reduced parameter vector \f$ \mathbf{\eta} \f$. */

    using x_t = Eigen::Matrix<double, k, 1>;                /*!< \brief Input vector type \f$k \times 1\f$ */
    using xs_t = Eigen::Matrix<double, k, -1>;              /*!< \brief Container type for the input data array */
    using u_t = Eigen::Matrix<double, l, 1>;                /*!< \brief Transformed input vector type \f$l \times 1\f$ */
    using P_t = Eigen::Matrix<double, k, k>;                /*!< \brief Covariance type of the input vector \f$k \times k\f$ */
    using Ps_t = std::vector<P_t>;                          /*!< \brief Container type for covariances \p P of the input data array */

    using z_t = Eigen::Matrix<double, lr, 1>;                       /*!< \brief Type of a reduced transformed input vector \f$l_r \times 1\f$ */
    using zs_t = Eigen::Matrix<double, lr, -1>;                     /*!< \brief Container type for an array of the reduced transformed input vectors \p z */
    using f_z_t = typename std::function<zs_t(const xs_t&)>;        /*!< \brief Function signature of the \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ mapping function */

    using dzdx_t = Eigen::Matrix<double, lr, k>;                    /*!< \brief Type of the jacobian matrix \f$ \mathbf{J} \mathbf{z}\left( \mathbf{x} \right) \f$, evaluated at \f$ \mathbf{x} \f$ */
    using dzdxs_t = std::vector<dzdx_t>;                            /*!< \brief Contained type for an array of the jacobian matrices */
    using f_dzdx_t = typename std::function<dzdx_t(const xs_t&)>;   /*!< \brief Function signature of the jacobian \f$ \mathbf{J} \mathbf{z}\left( \mathbf{x} \right) \f$ */

    using theta_t = Eigen::Matrix<double, l, 1>;            /*!< \brief Parameter vector type \f$l \times 1\f$ */
    using eta_t = z_t;                                      /*!< \brief Reduced parameter vector type \f$l_r \times 1\f$ */

  private:
    using A_t = Eigen::Matrix<double, lr, lr>;          /*!< \brief Type of the helper matrix \f$m \mathbf{A} 1\f$, \f$l_r \times l_r\f$ */
    using B_t = A_t;                                    /*!< \brief Type of the helper matrix \f$m \mathbf{B} 1\f$, \f$l_r \times l_r\f$ */
    using Bs_t = std::vector<B_t>;                      /*!< \brief Container type for an array of matrices \p B, corresponding to the input data */
    using M_t = A_t;                                    /*!< \brief Type of the matrix \f$m \mathbf{M} 1\f$, used in the generalized eigen vector problem, \f$l_r \times l_r\f$ */
    using N_t = A_t;                                    /*!< \brief Type of the matrix \f$m \mathbf{N} 1\f$, used in the generalized eigen vector problem, \f$l_r \times l_r\f$ */
    using betas_t = Eigen::Matrix<double, 1, -1>;       /*!< \brief Container type for an array of coefficients \p beta, corresponding to the input data */
    using ms_t = std::chrono::milliseconds;
    
    //}

  public:
      /* constructor //{ */
      /*!
        * \brief Convenience default constructor.
        *
        * \warning This constructor should not be used if applicable. If used, the main constructor has to be called afterwards,
        *          before using this class, otherwise the RHEIV object is invalid (not initialized).
        */
        RHEIV()
          :
            m_initialized(false),
            m_f_z(),
            m_f_dzdx(),
            m_min_dtheta(),
            m_max_its(),
            m_timeout(),
            m_debug_nth_it(),
            m_ALS_theta_set(false),
            m_last_theta_set(false)
        {};

      /*!
        * \brief The main constructor.
        *
        * The \p dzdx parameter gives the relation between \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ and \f$ \mathbf{x} \f$. It is the full jacobian matrix
        * \f$ \partial_{\mathbf{x}} \mathbf{z}\left( \mathbf{x} \right) \f$.
        *
        * The optimization algorithm is iterative with two possible stopping conditions:
        * - if change of the estimate of \f$ \mathbf{\theta} \f$ between two iterations is smaller than a defined threshold, or
        * - if a maximal number of iterations was reached.
        *
        * \param f_z                   the mapping function \f$ \mathbf{z}\left( \mathbf{x} \right) \f$.
        * \param f_dzdx                a function, returning the jacobian matrix of partial derivations of \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ by \f$ \mathbf{x} \f$, evaluated at \f$ \mathbf{x} \f$.
        * \param min_dtheta            if the difference of \f$ \mathbf{\theta}_{k} \f$ and \f$ \mathbf{\theta}_{k-1} \f$ is smaller than this number, the iteration is stopped.
        * \param max_its               if the iteration is stopped after this number of iterations.
        */
        RHEIV(const f_z_t& f_z, const f_dzdx_t& f_dzdx, const double min_dtheta = 1e-15, const unsigned max_its = 100)
          : 
            m_initialized(true),
            m_f_z(f_z),
            m_f_dzdx(f_dzdx),
            m_min_dtheta(min_dtheta),
            m_max_its(max_its),
            m_timeout(ms_t::zero()),
            m_debug_nth_it(-1),
            m_ALS_theta_set(false),
            m_last_theta_set(false)
        {};

      /*!
        * \brief The main constructor.
        *
        * The \p dzdx parameter gives the relation between \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ and \f$ \mathbf{x} \f$. It is the full jacobian matrix
        * \f$ \partial_{\mathbf{x}} \mathbf{z}\left( \mathbf{x} \right) \f$.
        *
        * The optimization algorithm is iterative with two possible stopping conditions:
        * - if change of the estimate of \f$ \mathbf{\theta} \f$ between two iterations is smaller than a defined threshold, or
        * - if a maximal number of iterations was reached.
        *
        * \param f_z                   the mapping function \f$ \mathbf{z}\left( \mathbf{x} \right) \f$.
        * \param f_dzdx                a function, returning the jacobian matrix of partial derivations of \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ by \f$ \mathbf{x} \f$, evaluated at \f$ \mathbf{x} \f$.
        * \param min_dtheta            if the difference of \f$ \mathbf{\theta}_{k} \f$ and \f$ \mathbf{\theta}_{k-1} \f$ is smaller than this number, the iteration is stopped.
        * \param max_its               if the iteration is stopped after this number of iterations.
        * \param timeout               if the calculation takes longer than this time, the iteration is stopped.
        * \param debug_nth_it          a debug message will be printed every \p debug_nth_it th iteration (negative number disables debug).
        */
        template <typename time_t>
        RHEIV(const f_z_t& f_z, const f_dzdx_t& f_dzdx, const double min_dtheta = 1e-15, const unsigned max_its = 100, const time_t& timeout = std::chrono::duration_cast<time_t>(ms_t::zero()), const int debug_nth_it = -1)
          : 
            m_initialized(true),
            m_f_z(f_z),
            m_f_dzdx(f_dzdx),
            m_min_dtheta(min_dtheta),
            m_max_its(max_its),
            m_timeout(std::chrono::duration_cast<ms_t>(timeout)),
            m_debug_nth_it(debug_nth_it),
            m_ALS_theta_set(false),
            m_last_theta_set(false)
        {}

      /*!
        * \brief A convenience constructor constructor.
        *
        * This constructor differs from the main one only in the parameters it takes. Instead of the function f_dzdx, it takes directly the dzdx matrix. This variant is meant to be used
        * for systems where the jacobian matrix \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ by \f$ \mathbf{x} \f$ does not depend on \f$ \mathbf{x} \f$.
        *
        * \param f_z                   the mapping function \f$ \mathbf{z}\left( \mathbf{x} \right) \f$.
        * \param dzdx                  the jacobian matrix of partial derivations of \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ by \f$ \mathbf{x} \f$.
        * \param min_dtheta            if the difference of \f$ \mathbf{\theta}_{k} \f$ and \f$ \mathbf{\theta}_{k-1} \f$ is smaller than this number, the iteration is stopped.
        */
        RHEIV(const f_z_t& f_z, const dzdx_t& dzdx, const double min_dtheta = 1e-15, const unsigned max_its = 100)
          : 
            m_initialized(true),
            m_f_z(f_z),
            // define a helper function which just returns the constant dzdx
            m_f_dzdx(
                {
                [dzdx](const x_t&)
                  {
                    return dzdx;
                  }
                }),
            m_min_dtheta(min_dtheta),
            m_max_its(max_its),
            m_timeout(ms_t::zero()),
            m_debug_nth_it(-1),
            m_ALS_theta_set(false),
            m_last_theta_set(false)
        {};

      /*!
        * \brief A convenience constructor constructor.
        *
        * This constructor differs from the main one only in the parameters it takes. Instead of the function f_dzdx, it takes directly the dzdx matrix. This variant is meant to be used
        * for systems where the jacobian matrix \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ by \f$ \mathbf{x} \f$ does not depend on \f$ \mathbf{x} \f$.
        *
        * \param f_z                   the mapping function \f$ \mathbf{z}\left( \mathbf{x} \right) \f$.
        * \param dzdx                  the jacobian matrix of partial derivations of \f$ \mathbf{z}\left( \mathbf{x} \right) \f$ by \f$ \mathbf{x} \f$.
        * \param min_dtheta            if the difference of \f$ \mathbf{\theta}_{k} \f$ and \f$ \mathbf{\theta}_{k-1} \f$ is smaller than this number, the iteration is stopped.
        * \param timeout               if the calculation takes longer than this time, the iteration is stopped.
        * \param max_its               if the iteration is stopped after this number of iterations.
        * \param debug_nth_it          a debug message will be printed every \p debug_nth_it th iteration (negative number disables debug).
        */
        template <typename time_t>
        RHEIV(const f_z_t& f_z, const dzdx_t& dzdx, const double min_dtheta = 1e-15, const unsigned max_its = 100, const time_t& timeout = std::chrono::duration_cast<time_t>(ms_t::zero()), const int debug_nth_it = -1)
          : 
            m_initialized(true),
            m_f_z(f_z),
            // define a helper function which just returns the constant dzdx
            m_f_dzdx(
                {
                [dzdx](const x_t&)
                  {
                    return dzdx;
                  }
                }),
            m_min_dtheta(min_dtheta),
            m_max_its(max_its),
            m_timeout(std::chrono::duration_cast<ms_t>(timeout)),
            m_debug_nth_it(debug_nth_it),
            m_ALS_theta_set(false),
            m_last_theta_set(false)
        {}
      //}

      /* fit() method //{ */
      /*!
        * \brief Fit the defined model to the provided data.
        *
        * The RHEIV iterative optimization algorithm will be applied to estimate optimal parameters of the model based on the provided data.
        *
        * \param xs the data points \f$ \mathbf{x}_i \f$.
        * \param Ps the corresponding covariance matrices\f$ \mathbf{P}_i \f$ .
        *
        * \returns  estimate of the parameter vector \f$ \mathbf{\theta} \f$.
        *
        * \warning  Note that length of \p xs and \p Ps must be the same!
        *
        */
        theta_t fit(const xs_t& xs, const Ps_t& Ps)
        {
          assert(m_initialized);
          assert((size_t)xs.cols() == Ps.size());
          const std::chrono::system_clock::time_point fit_start = std::chrono::system_clock::now();
      
          const zs_t zs = m_f_z(xs);
          const dzdxs_t dzdxs = precalculate_dxdzs(xs, m_f_dzdx);
      
          // Find initial conditions through ALS
          m_ALS_theta = fit_ALS_impl(zs);
          m_last_theta = m_ALS_theta;
          m_ALS_theta_set = m_last_theta_set = true;
          eta_t eta = m_ALS_theta.template block<lr, 1>(0, 0);
      
          for (unsigned it = 0; it < m_max_its; it++)
          {
            const std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            const auto fit_dur = now - fit_start;
            if (m_timeout > ms_t::zero() && fit_dur > m_timeout)
            {
              if (m_debug_nth_it > 0)
                std::cerr << "[RHEIV]: Ending at iteration " << it << " (max " << m_max_its << ") - timed out." << std::endl;
              break;
            }
            const theta_t prev_theta = m_last_theta;
            const auto [M, N, zc] = calc_MN(eta, zs, Ps, dzdxs);
            eta = calc_min_eigvec(M, N);
            m_last_theta = calc_theta(eta, zc);
            const double dtheta = calc_dtheta(prev_theta, m_last_theta);
            if (m_debug_nth_it > 0 && it % m_debug_nth_it == 0)
              std::cout << "[RHEIV]: iteration " << it << " (max " << m_max_its << "), dtheta: " << dtheta << " (min " << m_min_dtheta << ") " << std::endl;
            if (dtheta < m_min_dtheta)
                break;
          }
          return m_last_theta;
        }

      /*!
        * \brief Fit the defined model to the provided data.
        *
        * The RHEIV iterative optimization algorithm will be applied to estimate optimal parameters of the model based on the provided data.
        * This is a conveinence overload, which takes the data points and covariances in a general container, supporting std::begin() and std::end().
        *
        * \param xs_begin begin iterator of a container, containing the data points \f$ \mathbf{x}_i \f$.
        * \param xs_end   end iterator of a container, containing the data points \f$ \mathbf{x}_i \f$.
        * \param Ps_begin begin iterator of a container, containing the corresponding covariance matrices\f$ \mathbf{P}_i \f$ .
        * \param Ps_end   end iterator of a container, containing the corresponding covariance matrices\f$ \mathbf{P}_i \f$ .
        *
        * \returns  estimate of the parameter vector \f$ \mathbf{\theta} \f$.
        *
        * \warning  Note that length of \p xs and \p Ps must be the same!
        *
        */
        template <class T_it1,
                  class T_it2>
        theta_t fit(const T_it1& xs_begin, const T_it1& xs_end, const T_it2& Ps_begin, const T_it2& Ps_end)
        {
          const xs_t xs = cont_to_eigen(xs_begin, xs_end);
          const Ps_t Ps = cont_to_vector(Ps_begin, Ps_end);
          const theta_t ret = fit(xs, Ps);
          return ret;
        }
      //}

      /* fit_ALS() method //{ */
      /*!
        * \brief Fit the defined model to the provided data using Algebraic Least Squares (not RHEIV).
        *
        * Instead of using the iterative RHEIV algorithm to fit the data, just use the simple ALS (which basically does eigenvector decomposition).
        * This can be useful in cases of degenerate data where the RHEIV algorithm fails or when covariances of the data are not available.
        *
        * \param xs the data points \f$ \mathbf{x}_i \f$.
        *
        * \returns  estimate of the parameter vector \f$ \mathbf{\theta} \f$.
        *
        */
        theta_t fit_ALS(const xs_t& xs)
        {
          assert(m_initialized);
      
          const zs_t zs = m_f_z(xs);
          m_ALS_theta = fit_ALS_impl(zs);
          m_ALS_theta_set = true;
          return m_ALS_theta;
        }
      //}

      /* get_last_estimate() method //{ */
      /*!
        * \brief Returns the last valid estimate of \f$ \mathbf{\theta} \f$.
        *
        * You can use this method in case of an exception to retreive the last valid estimate of the parameter vector
        * before the crash, which is sometimes useful.
        *
        * \returns  the last valid estimate of the parameter vector \f$ \mathbf{\theta} \f$.
        *
        * \warning  The fit() method must be called prior to attempting to get the last valid estimate!
        *
        */
        theta_t get_last_estimate() const
        {
          assert(m_last_theta_set);
          return m_last_theta;
        }
      //}

      /* get_ALS_estimate() method //{ */
      /*!
        * \brief Returns the Algebraic Least Squares estimate of \f$ \mathbf{\theta} \f$.
        *
        * You can use this method in case of an exception to retreive the Algebraic Least Squares estimate of the parameter vector,
        * which is more stable than the RHEIV algorithm, and can be used as a more rough estimate of the parameters.
        *
        * \returns  the ALS estimate of the parameter vector \f$ \mathbf{\theta} \f$.
        *
        * \warning  The fit() method must be called prior to attempting to get the last ALS estimate!
        *
        */
        theta_t get_ALS_estimate() const
        {
          assert(m_ALS_theta_set);
          return m_ALS_theta;
        }
      //}

  private:
    bool m_initialized;

  private:
    f_z_t m_f_z;
    f_dzdx_t m_f_dzdx;
    double m_min_dtheta;
    unsigned m_max_its;
    ms_t m_timeout;
    int m_debug_nth_it;

  private:
    bool m_ALS_theta_set;
    theta_t m_ALS_theta;
    bool m_last_theta_set;
    theta_t m_last_theta;

  private:
    /* calc_MN() method //{ */
    std::tuple<M_t, N_t, z_t> calc_MN(const eta_t& eta, const zs_t& zs, const Ps_t& Ps, const dzdxs_t& dzdxs) const
    {
      const int n = zs.cols();
    
      const Bs_t Bs = calc_Bs(Ps, dzdxs);
      const betas_t betas = calc_betas(eta, Bs);
      const auto [zrs, zc] = reduce_zs(zs, betas);
    
      M_t M = M_t::Zero();
      N_t N = N_t::Zero();
      for (int it = 0; it < n; it++)
      {
          const double beta = betas(it);
          const z_t& zr = zrs.col(it);
          const B_t& B = Bs.at(it);
          const A_t A = calc_A(zr);
          M += A*beta;
          N += (eta.transpose()*A*eta)*B*(beta*beta);
      }
      return {M, N, zc};
    }
    //}

    /* calc_Bs() method //{ */
    Bs_t calc_Bs(const Ps_t& Ps, const dzdxs_t& dzdxs) const
    {
      const int n = Ps.size();
      Bs_t Bs;
      Bs.reserve(n);
      for (int it = 0; it < n; it++)
      {
        const P_t& P = Ps.at(it);
        const dzdx_t& dzdx = dzdxs.at(it);
        const B_t B = dzdx*P*dzdx.transpose();
        Bs.push_back(B);
      }
      return Bs;
    }
    //}

    /* calc_betas() method //{ */
    betas_t calc_betas(const eta_t& eta, const Bs_t& Bs) const
    {
      const int n = Bs.size();
      betas_t betas(n);
      for (int it = 0; it < n; it++)
      {
          const B_t& B = Bs.at(it);
          const double beta = 1.0/(eta.transpose()*B*eta);
          betas(it) = beta;
      }
      return betas;
    }
    //}

    /* calc_A() method //{ */
    A_t calc_A(const z_t& z) const
    {
      return z*z.transpose();
    }
    //}

    /* calc_dtheta() method //{ */
    double calc_dtheta(const theta_t& th1, const theta_t& th2) const
    {
      return std::min( (th1 - th2).norm(), (th1 + th2).norm() );
    }
    //}

    /* calc_theta() method //{ */
    theta_t calc_theta(const eta_t& eta, const z_t& zc) const
    {
      const double alpha = -zc.transpose()*eta;
      const theta_t theta( (theta_t() << eta, alpha).finished().normalized() );
      return theta;
    }
    //}

    /* calc_min_eigvec() method //{ */
    eta_t calc_min_eigvec(const M_t& M, const N_t& N) const
    {
      const Eigen::GeneralizedSelfAdjointEigenSolver<M_t> es(M, N);
      if (es.info() != Eigen::Success)
        throw eigenvector_exception();
      // eigenvalues are sorted in increasing order when using the GeneralizedSelfAdjointEigenSolver as per Eigen documentation
      const eta_t evec = es.eigenvectors().col(0).normalized(); // altough the Eigen documentation states that this vector should already be normalized, it isn't!!
      return evec;
    }
    //}

    /* calc_min_eigvec() method //{ */
    eta_t calc_min_eigvec(const M_t& M) const
    {
      const Eigen::SelfAdjointEigenSolver<M_t> es(M);
      if (es.info() != Eigen::Success)
        throw eigenvector_exception();
      // eigenvalues are sorted in increasing order when using the SelfAdjointEigenSolver as per Eigen documentation
      const eta_t evec = es.eigenvectors().col(0).normalized(); // altough the Eigen documentation states that this vector should already be normalized, it isn't!!
      return evec;
    }
    //}

    /* precalculate_dxdzs() method //{ */
    dzdxs_t precalculate_dxdzs(const xs_t& xs, const f_dzdx_t& f_dzdx)
    {
      const int n = xs.cols();
      dzdxs_t ret;
      ret.reserve(n);
      for (int it = 0; it < n; it++)
      {
        const x_t& x = xs.col(it);
        ret.push_back(f_dzdx(x));
      }
      return ret;
    }
    //}

    /* calc_centroid() method //{ */
    z_t calc_centroid(const zs_t& zs, const betas_t& betas) const
    {
      const z_t zc = (zs.array().rowwise() * betas.array()).rowwise().sum()/betas.sum();
      return zc;
    }
    //}

    /* reduce_zs() method //{ */
    std::pair<zs_t, z_t> reduce_zs(const zs_t& zs, const betas_t& betas) const
    {
      const z_t zc = calc_centroid(zs, betas);
      const zs_t zrs = zs.colwise() - zc;
      return {zrs, zc};
    }
    //}

    /* cont_to_eigen() method //{ */
    template <typename T_it>
    xs_t cont_to_eigen(const T_it& begin, const T_it& end)
    {
      const auto n = end-begin;
      xs_t ret(n_states, n);
      size_t i = 0;
      for (T_it it = begin; it != end; it++)
      {
        ret.template block<n_states, 1>(0, i) = *it;
        i++;
      }
      return ret;
    }
    //}

    /* cont_to_vector() method //{ */
    template <typename T_it>
    Ps_t cont_to_vector(const T_it& begin, const T_it& end)
    {
      const auto n = end-begin;
      Ps_t ret(n);
      size_t i = 0;
      for (T_it it = begin; it != end; it++)
      {
        ret.at(i) = *it;
        i++;
      }
      return ret;
    }
    //}


// --------------------------------------------------------------
// |           Algebraic Least Squares-related methods          |
// --------------------------------------------------------------

    z_t calc_centroid(const zs_t& zs) const
    {
      return zs.rowwise().mean();
    }

    std::pair<zs_t, z_t> reduce_zs(const zs_t& zs) const
    {
      const z_t zc = calc_centroid(zs);
      const zs_t zrs = zs.colwise() - zc;
      return {zrs, zc};
    }

    theta_t fit_ALS_impl(const zs_t& zs) const
    {
      const auto [zrs, zc] = reduce_zs(zs);
      M_t M = M_t::Zero();
      for (int it = 0; it < zrs.cols(); it++)
      {
        const z_t& z = zrs.col(it);
        const A_t A = calc_A(z);
        M += A;
      }
      const eta_t eta = calc_min_eigvec(M);
      const theta_t theta = calc_theta(eta, zc);
      return theta;
    }

  };
  //}
  
}

#endif // HEIV_H
