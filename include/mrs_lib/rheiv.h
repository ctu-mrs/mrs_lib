// clang: MatousFormat
/**  \file
     \brief Defines RHEIV and related stuff for surface fitting to points with known covariances according to \cite FNSandHEIV.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef HEIV_H
#define HEIV_H

#include <Eigen/Dense>
#include <iostream>

namespace mrs_lib
{

  /* class RHEIV //{ */
  /**
  * \brief Implementation of the Reduced Heteroscedastic Errors In Variables surface fitting algorithm \cite FNSandHEIV.
  *
  * Example usage:
  * \include src/heiv/example.cpp
  *
  */
  template <int n_states, int n_params>
  class RHEIV
  {
  public:
    /* RHEIV definitions (typedefs, constants etc) //{ */

    static const int k = n_states;            /*!< \brief Length of the state vector of the system. */
    static const int l = n_params;            /*!< \brief Length of the state vector of the system. */
    static const int lr = l - 1;            /*!< \brief Length of the input vector of the system. */

    using x_t = Eigen::Matrix<double, k, 1>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using P_t = Eigen::Matrix<double, k, k>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using Ps_t = std::vector<P_t>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using dzdx_t = Eigen::Matrix<double, lr, k>;                /*!< \brief Input vector type \f$m \times 1\f$ */

    using u_t = Eigen::Matrix<double, l, 1>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using us_t = Eigen::Matrix<double, l, -1>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using z_t = Eigen::Matrix<double, lr, 1>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using zs_t = Eigen::Matrix<double, lr, -1>;                /*!< \brief Input vector type \f$m \times 1\f$ */

    using theta_t = u_t;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using nu_t = z_t;                /*!< \brief Input vector type \f$m \times 1\f$ */

    using A_t = Eigen::Matrix<double, lr, lr>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using B_t = A_t;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using Bs_t = std::vector<B_t>;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using M_t = A_t;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using N_t = A_t;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using betas_t = Eigen::Matrix<double, 1, -1>;

    //}

  public:
  /*!
    * \brief Convenience default constructor.
    *
    * This constructor should not be used if applicable. If used, the main constructor has to be called afterwards,
    * before using this class, otherwise the LKF object is invalid (not initialized).
    */
    RHEIV(){};

  /*!
    * \brief The main constructor.
    *
    * \param A             The state transition matrix.
    */
    RHEIV(const dzdx_t& dzdx, const double min_dnu = 1e-15, const unsigned max_its = 100) : m_dzdx(dzdx), m_min_dnu(min_dnu), m_max_its(max_its) {};

    theta_t fit(const us_t& us, const Ps_t& Ps)
    {
      const int n = us.cols();
      const zs_t zs = us.block(0, 0, lr, n);

      // Find initial conditions through ALS
      const auto [theta0, zc0] = fit_ALS(zs);
      nu_t nu = theta0.template block<lr, 1>(0, 0);
      z_t last_zc = zc0;
      
      for (unsigned it = 0; it < m_max_its; it++)
      {
          std::cout << "-------------------------------------" << std::endl;
          std::cout << "iteration: " << it << std::endl;
          const nu_t prev_nu = nu;
          std::cout << "pre-nu:" << std::endl << nu << std::endl;
          const auto [M, N, zc] = calc_MN(nu, zs, Ps, m_dzdx);
          std::cout << "M:" << std::endl << M << std::endl;
          std::cout << "N:" << std::endl << N << std::endl;
          last_zc = zc;
          nu = calc_min_eigvec(M, N);
          std::cout << "post-nu:" << std::endl << nu << std::endl;
          if ((prev_nu - nu).norm() < m_min_dnu)
              break;
      }
      const theta_t theta = calc_theta(nu, last_zc);
      return theta;
    }

  protected:
    dzdx_t m_dzdx;
    double m_min_dnu;
    unsigned m_max_its;

  protected:
    std::tuple<M_t, N_t, z_t> calc_MN(const nu_t& nu, const zs_t& zs, const Ps_t& Ps, const dzdx_t& dzdx)
    {
      const int n = zs.cols();
    
      const Bs_t Bs = calc_Bs(Ps, dzdx);
      const betas_t betas = calc_betas(nu, Bs);
      const auto [zrs, zc] = reduce_zs(zs, betas);
      
      M_t M = M_t::Zero();
      N_t N = N_t::Zero();
      for (int it = 0; it < n; it++)
      {
          const double beta = betas(it);
          const z_t& zr = zrs.col(it);
          const B_t& B = Bs.at(it);
          const A_t A = calc_A(zr);
          M += A/beta;
          N += (nu.transpose()*A*nu)*B/(beta*beta);
      }
      return {M, N, zc};
    }

    Bs_t calc_Bs(const Ps_t& Ps, const dzdx_t& dzdx)
    {
      const int n = Ps.size();
      Bs_t Bs;
      Bs.reserve(n);
      for (int it = 0; it < n; it++)
      {
        const P_t& P = Ps.at(it);
        const B_t B = dzdx*P*dzdx.transpose();
        Bs.push_back(B);
      }
      return Bs;
    }

    betas_t calc_betas(const nu_t& nu, const Bs_t& Bs)
    {
      const int n = Bs.size();

      betas_t betas(n);
      for (int it = 0; it < n; it++)
      {
          const B_t& B = Bs.at(it);
          const double beta = 1/(nu.transpose()*B*nu);
          betas(it) = beta;
      }
      return betas;
    }

    A_t calc_A(const z_t& z)
    {
      return z*z.transpose();
    }

    theta_t calc_theta(const nu_t& nu, const z_t& zc)
    {
      const double alpha = -zc.transpose()*nu;
      const theta_t theta( (theta_t() << nu, alpha).finished().normalized());
      return theta;
    }

    nu_t calc_min_eigvec(const M_t& M, const N_t& N)
    {
      const Eigen::GeneralizedSelfAdjointEigenSolver<M_t> es(M, N);
      if (es.info() != Eigen::Success)
        // TODO: throw an exception
        std::cerr << "GeneralizedSelfAdjointEigenSolver failed: " << es.info() << std::endl; 
      // eigenvalues are sorted in increasing order when using the GeneralizedSelfAdjointEigenSolver as per Eigen documentation
      const nu_t evec = es.eigenvectors().col(0).normalized(); // altough the Eigen documentation states that this vector should already be normalized, it isn't!!
      /* std::cout << "GeneralizedSelfAdjointEigenSolver eigenvec norm: " << evec.norm() << std::endl; */
      return evec;
    }

    nu_t calc_min_eigvec(const M_t& M)
    {
      const Eigen::SelfAdjointEigenSolver<M_t> es(M);
      if (es.info() != Eigen::Success)
        std::cerr << "SelfAdjointEigenSolver failed: " << es.info() << std::endl; 
      // eigenvalues are sorted in increasing order when using the SelfAdjointEigenSolver as per Eigen documentation
      const nu_t evec = es.eigenvectors().col(0).normalized(); // altough the Eigen documentation states that this vector should already be normalized, it isn't!!
      std::cout << "SelfAdjointEigenSolver eigenvec norm: " << evec.norm() << std::endl;
      return evec;
    }

    z_t calc_centroid(const zs_t& zs, const betas_t& betas)
    {
      const z_t zc = (zs.array().rowwise() * betas.array()).rowwise().sum()/betas.sum();
      return zc;
    }

    std::pair<zs_t, z_t> reduce_zs(const zs_t& zs, const betas_t& betas)
    {
      const z_t zc = calc_centroid(zs, betas);
      const zs_t zrs = zs.colwise() - zc;
      return {zrs, zc};
    }

// --------------------------------------------------------------
// |           Algebraic Least Squares-related methods          |
// --------------------------------------------------------------

    z_t calc_centroid(const zs_t& zs)
    {
      return zs.rowwise().mean();
    }

    std::pair<zs_t, z_t> reduce_zs(const zs_t& zs)
    {
      const z_t zc = calc_centroid(zs);
      const zs_t zrs = zs.colwise() - zc;
      return {zrs, zc};
    }

    std::pair<theta_t, z_t> fit_ALS(const zs_t& zs)
    {
      const auto [zrs, zc] = reduce_zs(zs);
      M_t M = M_t::Zero();
      for (int it = 0; it < zrs.cols(); it++)
      {
        const z_t& z = zrs.col(it);
        const A_t A = calc_A(z);
        M += A;
      }
      const nu_t nu = calc_min_eigvec(M);
      theta_t theta = calc_theta(nu, zc);
      return {theta, zc};
    }

  };
  //}
  
}

#endif // HEIV_H
