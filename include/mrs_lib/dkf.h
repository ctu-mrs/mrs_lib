// clang: MatousFormat
/**  \file LKF
     \brief Defines LKF - a class, implementing the Linear Kalman Filter \cite LKF, as well as a few specialized variants.
     \author Matouš Vrba - vrbamato@fel.cvut.cz
 */

#ifndef DKF_H
#define DKF_H

#include <mrs_lib/lkf.h>
#include <mrs_lib/geometry/misc.h>
#include <iostream>

namespace mrs_lib
{

  /* class DKF //{ */
  /**
  * \brief Implementation of the Degenerate measurement Linear Kalman filter.
  *
  * \tparam n_states         number of states of the system (length of the \f$ \mathbf{x} \f$ vector).
  * \tparam n_inputs         number of inputs of the system (length of the \f$ \mathbf{u} \f$ vector).
  * \tparam n_measurements   number of measurements of the system (length of the \f$ \mathbf{z} \f$ vector).
  *
  */
  template <int n_states, int n_inputs>
  class DKF : public LKF<n_states, n_inputs, -1>
  {
  public:
    /* DKF definitions (typedefs, constants etc) //{ */
    using Base_class = LKF<n_states, n_inputs, -1>;            /*!< \brief Base class of this class. */

    static constexpr int n = Base_class::n;              /*!< \brief Length of the state vector of the system. */
    static constexpr int m = Base_class::m;              /*!< \brief Length of the input vector of the system. */
    static constexpr int p = Base_class::p;              /*!< \brief Length of the measurement vector of the system. */

    using x_t = typename Base_class::x_t;                /*!< \brief State vector type \f$n \times 1\f$ */
    using u_t = typename Base_class::u_t;                /*!< \brief Input vector type \f$m \times 1\f$ */
    using z_t = typename Base_class::z_t;                /*!< \brief Measurement vector type \f$p \times 1\f$ */
    using P_t = typename Base_class::P_t;                /*!< \brief State uncertainty covariance matrix type \f$n \times n\f$ */
    using R_t = typename Base_class::R_t;                /*!< \brief Measurement noise covariance matrix type \f$p \times p\f$ */
    using Q_t = typename Base_class::Q_t;                /*!< \brief Process noise covariance matrix type \f$n \times n\f$ */
    using statecov_t = typename Base_class::statecov_t;  /*!< \brief Helper struct for passing around the state and its covariance in one variable */

    typedef Eigen::Matrix<double, n, n> A_t;  /*!< \brief System transition matrix type \f$n \times n\f$ */
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
    DKF(){};

  /*!
    * \brief The main constructor.
    *
    * \param A             The state transition matrix.
    * \param B             The input to state mapping transition matrix.
    * \param H             The state to measurement mapping transition matrix.
    */
    DKF(const A_t& A, const B_t& B) : Base_class(A, B, {}) {};

    /* correctLine() method //{ */
  /*!
    * \brief Applies the correction (update, measurement, data) step of the Kalman filter.
    *
    * This method applies the linear Kalman filter correction step to the state and covariance
    * passed in \p sc using a measurement in the form of a line direcition vector, a point on the line and a perpendicular variance.
    * The updated state and covariance after the correction step is returned.
    *
    * \param sc             The state and covariance to which the correction step is to be applied.
    * \param line_origin    A point lying on the measurement line
    * \param line_direction A vector defining the span of the measurement line
    * \param line_variance  Variance defining the uncertainty of the measured state in the direction perpendicular to the measurement line. The uncertainty in the parallel direciton is assumed to be infinite for this case of DKF.
    * \return               The state and covariance after the correction update.
    */
    virtual std::enable_if_t<((n > 3) || (n == Eigen::Dynamic)), statecov_t> correctLine(const statecov_t& sc, const pt3_t& line_origin, const vec3_t& line_direction, const double line_variance) const
    {
      assert(line_direction.norm() > 0.0);

      using M_t = Eigen::Matrix<double, 3, n>;
      using W_t = Eigen::Matrix<double, 3, 1>;
      using N_t = Eigen::Matrix<double, 3, 2>;
      using o_t = Eigen::Matrix<double, 3, 1>;
      using R_t = Eigen::Matrix<double, 2, 2>;

      const M_t M = M_t::Identity(3, sc.P.cols());
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
      const R_t R = line_variance * N.transpose() * N;

      return this->correction_impl(sc, z, R, H);
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
    * \param sc             The state and covariance to which the correction step is to be applied.
    * \param plane_origin   A point lying on the measurement plane
    * \param plane_normal   The normal vector of the measurement plane
    * \param plane_variance Variance defining the uncertainty of the measured state in the direction perpendicular to the measurement plane. The uncertainty in the span of the plane is assumed to be infinite for this case of DKF.
    * \return               The state and covariance after the correction update.
    */
    virtual std::enable_if_t<((n > 3) || (n == Eigen::Dynamic)), statecov_t> correctPlane(const statecov_t& sc, const pt3_t& plane_origin, const vec3_t& plane_normal, const double plane_variance) const
    {
      assert(plane_normal.norm() > 0.0);

      // we don't need W, since the plane is minimally defined by its origin and normal, where the normal is a basis for its null space
      using M_t = Eigen::Matrix<double, 3, n>;
      using N_t = Eigen::Matrix<double, 3, 1>;
      using o_t = Eigen::Matrix<double, 3, 1>;
      using R_t = Eigen::Matrix<double, 1, 1>;

      const M_t M = M_t::Identity(3, sc.P.cols());
      const o_t o = plane_origin;

      const N_t N = plane_normal.normalized(); //works for plane
      const z_t z = N.transpose() * o;
      const H_t H = N.transpose() * M;
      const R_t R = (R_t() << plane_variance).finished(); //R is a scalar here

      return this->correction_impl(sc, z, R, H);
    };
    //}
  };
  //}

}  // namespace mrs_lib

#endif // DKF_H
