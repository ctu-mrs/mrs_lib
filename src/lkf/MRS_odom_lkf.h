#include 

namespace mrs_lib
{
  /* specialized Lkf declaration for MRS odom //{ */
  class MRS_odom_lkf {
    public:
      /* definitions, declarations etc. //{ */
      static const int n = Model_mrs_odom::n;
      static const int m = Model_mrs_odom::m;
      static const int p = Model_mrs_odom::p;
      
      using x_t = typename Model_mrs_odom::x_t;
      using u_t = typename Model_mrs_odom::u_t;
      using z_t = typename Model_mrs_odom::z_t;
      using P_t = typename Model_mrs_odom::P_t;
      using R_t = typename Model_mrs_odom::R_t;
      using statecov_t = typename Model_mrs_odom::statecov_t;
      using A_t = typename Model_mrs_odom::A_t;  // measurement mapping p*n
      using B_t = typename Model_mrs_odom::B_t;  // process covariance n*n
      using H_t = typename Model_mrs_odom::H_t;  // measurement mapping p*n
      using Q_t = typename Model_mrs_odom::Q_t;  // process covariance n*n
      //}
  
    public:
    
      // return estimated states
      x_t getStates() const;
    
      // reset the filter (with new particular state vector)
      void reset(const x_t& x);
    
      // set new measurement and its covariance
      void setMeasurement(const z_t& z, const R_t& R);
    
      // set new measurement
      void setMeasurement(const z_t& z);
    
      // set new input vector
      void setInput(const u_t& u);
    
      // return n-th states of the estimate state vector
      double getState(const int num) const;
    
      // get the covariance matrix
      P_t getCovariance(void) const;
    
      // get main matrix
      A_t getA(void) const;
    
      // set process noise
      void setQ(const Q_t& Q);
    
      // set covariance
      void setCovariance(const P_t& cov);
    
      // set n-th states of the estimate state vector
      void setState(const int num, const double value);
    
      // set all states of the estimate state vector
      void setStates(const Eigen::VectorXd& states);
    
      // do iteration of the filter
      statecov_t iterate(double dt);
    
      // iterate without the correction phase
      statecov_t iterateWithoutCorrection(double dt);
    
      // do just the correction
      statecov_t doCorrection();
    
      /* /1* methods for performance comparison (unused in release) //{ *1/ */
      /* void correctionImpl(); */
      /* void correctionImpl2(); */
      /* void correctionImpl3(); */
      /* void correctionImpl4(); */
      /* //} */
    
    private:
      Model_mrs_odom m_model;

      x_t m_last_x;    // last state vector
      P_t m_last_P;    // last state covariance
    
      z_t m_last_z;    // the last measurement
      R_t m_last_R;    // last measurement covariance
    
      u_t m_last_u;    // last system input vector
    
      mutable std::mutex lkf_mutex;
  };
  //}

};
