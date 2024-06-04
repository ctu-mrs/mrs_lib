// clang: MatousFormat
/**  \file UMDKF
     \brief Defines UMDKF - a class, implementing the Degenerate Kalman Filter \cite with specialized correction rule for measurements with uniform error distributions.

     This correction technique is derived from the paper:
     SEQUENTIAL FILTERING IN THE PRESENCE OF UNIFORM MEASUREMENT ERRORS
     by
     James S. McCabe

     \author Viktor Walter - viktor.walter@fel.cvut.cz
 */

#ifndef UMDKF_H
#define UMDKF_H

#include <mrs_lib/lkf.h>
#include <mrs_lib/geometry/misc.h>
#include <iostream>

namespace mrs_lib
{

  /* class UMDKF //{ */
  /**
  * \brief Implementation of the Degenerate measurement Linear Kalman filter with special correction rule for assumed uniform distribution of error.
  *
  * \tparam n_states         number of states of the system (length of the \f$ \mathbf{x} \f$ vector).
  * \tparam n_inputs         number of inputs of the system (length of the \f$ \mathbf{u} \f$ vector).
  * \tparam n_measurements   number of measurements of the system (length of the \f$ \mathbf{z} \f$ vector).
  *
  */
  template <int n_states, int n_inputs>
  class UMDKF : public LKF<n_states, n_inputs, -1>
  {
  public:
    /* UMDKF definitions (typedefs, constants etc) //{ */
    using Base_class = LKF<n_states, n_inputs, -1>;            /*!< \brief Base class of this class. */

    static constexpr int n = Base_class::n;              /*!< \brief Length of the state vector of the system. */
    static constexpr int m = Base_class::m;              /*!< \brief Length of the input vector of the system. */
    static constexpr int p = Base_class::p;              /*!< \brief Length of the measurement vector of the system. */

    using x_t = typename Base_class::x_t;                /*!< \brief State vector type \f$n \times 1\f$ */
    using u_t = typename Base_class::u_t;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using z_t = typename Base_class::z_t;                /*!< \brief Measurement vector type \f$p \times 1\f$ */
    using P_t = typename Base_class::P_t;                /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
    using A_t = typename Base_class::A_t;                /*!< \brief Vector of measurement span half-sides */
    using Q_t = typename Base_class::Q_t;                /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
    using statecov_t = typename Base_class::statecov_t;  /*!< \brief Helper struct for passing around the state and its covariance in one variable */

    typedef Eigen::Matrix<double, n, n> a_t;  /*!< \brief System transition matrix type \f$n \times n\f$ */
    typedef Eigen::Matrix<double, n, m> B_t;  /*!< \brief Input to state mapping matrix type \f$n \times m\f$ */
    typedef Eigen::Matrix<double, p, n> H_t;  /*!< \brief State to measurement mapping matrix type \f$p \times n\f$ */
    typedef Eigen::Matrix<double, n, p> K_t;  /*!< \brief Kalman gain matrix type \f$n \times p\f$ */

    using mat2_t = Eigen::Matrix<double, 2, 2>;
    using mat3_t = Eigen::Matrix<double, 3, 3>;
    using pt3_t = mrs_lib::geometry::vec3_t;
    using pt2_t = mrs_lib::geometry::vec2_t;
    using vec3_t = mrs_lib::geometry::vec3_t;
    //}

  public:
  /*!
    * \brief Convenience default constructor.
    *
    * This constructor should not be used if applicable. If used, the main constructor has to be called afterwards,
    * before using this class, otherwise the LKF object is invalid (not initialized).
    */
    UMDKF(){};

  /*!
    * \brief The main constructor.
    *
    * \param A             The state transition matrix.
    * \param B             The input to state mapping transition matrix.
    * \param H             The state to measurement mapping transition matrix.
    */
    UMDKF(const A_t& A, const B_t& B) : Base_class(A, B, {}) {};

    /* correctLine() method //{ */
  /*!
    * \brief Applies the correction (update, measurement, data) step of the Kalman filter.
    *
    * This method applies the linear Kalman filter correction step to the state and covariance
    * passed in \p sc using a measurement in the form of a line direcition vector, a point on the line and a perpendicular variance.
    * The updated state and covariance after the correction step is returned.
    *
    * \param sc              The state and covariance to which the correction step is to be applied.
    * \param line_origin     A point lying on the measurement line
    * \param line_direction  A vector defining the span of the measurement line
    * \param line_halfwidth  Variance defining the uncertainty of the measured state in the direction perpendicular to the measurement line. The uncertainty in the parallel direciton is assumed to be infinite for this case of UMDKF.
    * \return                The state and covariance after the correction update.
    */
    virtual std::enable_if_t<(n > 3), statecov_t> correctLine(const statecov_t& sc, const pt3_t& line_origin, const vec3_t& line_direction, const double line_halfwidth) const
    {
      assert(line_direction.norm() > 0.0);

      using M_t = Eigen::Matrix<double, 3, n>;
      using W_t = Eigen::Matrix<double, 3, 1>;
      using N_t = Eigen::Matrix<double, 3, 2>;
      using o_t = Eigen::Matrix<double, 3, 1>;
      using a_t = Eigen::Matrix<double, 2, 1>;

      const M_t M = M_t::Identity();
      const W_t W = line_direction;
      const o_t o = line_origin;

      // doesn't work - the kernel is always zero for some reason
      /* const Eigen::FullPivLU<W_t> lu(W); */
      /* const N_t N = lu.kernel(); */
      // works for a line measurement
      const mat3_t rot = mrs_lib::geometry::rotationBetween(W_t::UnitX(), W);
      // the first column should have the same direction as W - we don't care about it,
      // take the second and third column vectors, those are the null space of W
      const N_t N = rot.block<3, 2>(0, 1);
      const z_t z = N.transpose() * o;
      const H_t H = N.transpose() * M;
      const a_t a = line_halfwidth * N.transpose() * N;

      return this->correction_impl(sc, z, A, H);
    };
    //}
    
    /* correctPlane() method //{ */
  /*!
    * \brief Applies the correction (update, measurement, data) step of the Kalman filter.
    *
    * This method applies the linear Kalman filter correction step to the state and covariance
    * passed in \p sc using a measurement in the form of a plane normal vector, a point on the plane and a perpendicular variance.
    * The updated state and covariance after the correction step is returned.
    *
    * \param sc              The state and covariance to which the correction step is to be applied.
    * \param plane_origin    A point lying on the measurement plane
    * \param plane_normal    The normal vector of the measurement plane
    * \param plane_halfwidth Variance defining the uncertainty of the measured state in the direction perpendicular to the measurement plane. The uncertainty in the span of the plane is assumed to be infinite for this case of UMDKF.
    * \return                The state and covariance after the correction update.
    */
    virtual std::enable_if_t<(n > 3), statecov_t> correctPlane(const statecov_t& sc, const pt3_t& plane_origin, const vec3_t& plane_normal, const double plane_halfwidth) const
    {
      assert(plane_normal.norm() > 0.0);

      // we don't need W, since the plane is minimally defined by its origin and normal, where the normal is a basis for its null space
      using M_t = Eigen::Matrix<double, 3, n>;
      using N_t = Eigen::Matrix<double, 3, 1>;
      using o_t = Eigen::Matrix<double, 3, 1>;
      using a_t = Eigen::Matrix<double, 1, 1>;

      const M_t M = M_t::Identity();
      const o_t o = plane_origin;

      const N_t N = plane_normal.normalized(); //works for plane
      const z_t z = N.transpose() * o;
      const H_t H = N.transpose() * M;
      const a_t a = (a_t() << plane_halfwidth).finished(); //a is a scalar here

      return this->correction_impl(sc, z, a, H);
    };
    //}
  //}
  
    /* correction_impl() method //{ */
    //This method is derived from the paper in the header.
    //Various re-arrangements of the math from the paper were perfomed to ensure better numerical stability.
    template<int check=n>
    typename std::enable_if<check >= 0, statecov_t>::type correction_impl(const statecov_t& sc, const z_t& z, const a_t& a, const H_t& H) const
    {
      double sq2 = sqrt(2);

      W = H*sc.P*H.transpose();
      S = W.sqrt();
      iS = S.inverse();

      auto sct = sc;
      zhm = (z-H*sc.x);
      if (((zhm.cwiseAbs() - 3*a) > 0).any()) { //if any (abs(zhm) > 3*a)
        ROS_WARN_STREAM("[" << ros::this_node::getName().c_str() << "]: The measurement is likely an outlier (>3 sigma), or the state was highly off! I will reset the state covariance to high value.");
        sct.P.setIdentity();
        sct.P *= 666;
        W = H*sct.P*H.transpose()
        S = W.sqrt();
        iS = S.inverse();
      }
      beta = iS*zhm;
      b = is*a;

      auto phi = erf((b-beta)/sq2) + erf((b+beta)/sq2)
      auto iphi = phi.cwiseInverse();

      auto bbb = b.cwiseProduct(beta);
      auto bsq = b.cwiseProduct(b);
      auto betasq = beta.cwiseProduct(beta);
      auto hd = (bsq+betasq)/2; //half of squared diagonal

      auto q = sqrt(8/pi).cwiseProduct(iphi);
      auto C = ((b-bbeta)/2).cwiseProduct(q);
      auto D = ((b+bbeta)/2).cwiseProduct(q);
      auto m = -(hd)+bbb;
      auto n = -(hd)-bbb;

      //TODO check beta and epsilon for previous definitions - they are yellow
      epsilon = 0.5*q.cwiseProduct(exp(m)-exp(n));

      pz = A.cwiseProduct(exp(m)) + B.cwiseProduct(exp(n)); //psi * zeta, re-arranged for numerical stability
      Epsilon = diag((pz)+(epsilon.^2));
      g = iS.transpose()*epsilon;
      G = iS.transpose()*Epsilon*iS;
      sc.x = sct.x + sct.P*H.transpose()*g;
      sc.P = sct.P - sct.P*H.transpose()*G*H*Pi;

      // the correction phase
      statecov_t ret;
      const K_t K = computeKalmanGain(sc, z, a, H);
      ret.x = sct.x + K * (z - (H * sc.x));
      ret.P = (P_t::Identity() - (K * H)) * sc.P;
      return ret;
    }
    //}
};

}  // namespace mrs_lib

#endif // UMDKF_H
