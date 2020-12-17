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
  class DKF : public LKF<3, -1, 2>
  {
  public:
    /* DKF definitions (typedefs, constants etc) //{ */
    using Base_class = LKF<n, m, p>;            /*!< \brief Base class of this class. */

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
    using vec3_t = mrs_lib::geometry::vec3_t;

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

    /* correct() method //{ */
  /*!
    * \brief Applies the correction (update, measurement, data) step of the Kalman filter.
    *
    * This method applies the linear Kalman filter correction step to the state and covariance
    * passed in \p sc using the measurement \p z and measurement noise \p R. The parameter \p param
    * is ignored in this implementation. The updated state and covariance after the correction step
    * is returned.
    *
    * \param sc          The state and covariance to which the correction step is to be applied.
    * \param z           The measurement vector to be used for correction.
    * \param R           The measurement noise covariance matrix to be used for correction.
    * \return            The state and covariance after the correction update.
    */
    virtual statecov_t correctLine(const statecov_t& sc, const pt3_t& line_origin, const vec3_t& line_direction, const double line_variance) const
    {
      assert(line_direction.norm() > 0.0);
      const vec3_t zunit {0.0, 0.0, 1.0};
      // rot is a rotation matrix, transforming from F to F'
      const mat3_t rot = mrs_lib::geometry::rotationBetween(line_direction, zunit);
      /* const mat3_t rotT = rot.transpose(); */

      const H_t Hprime = rot.block<2, 2>(0, 0);

      const pt3_t oprime = rot*line_origin;
      const z_t z = oprime.block<2, 1>(0, 0);

      const R_t R = line_variance*R_t::Identity();
      return correction_impl(sc, z, R, Hprime);
    };
    //}
  };
  //}

}  // namespace mrs_lib

#endif // DKF_H
