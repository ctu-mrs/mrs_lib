// clang: MatousFormat
/**  \file
     \brief Example file for the Repredictor implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib repredictor_example`.

     See \ref repredictor/example.cpp.
 */

/**  \example "repredictor/example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib repredictor_example`.
 */

// Include the Repredictor header
#include <mrs_lib/repredictor.h>
// As a model, we'll use a LKF variant
#include <mrs_lib/lkf.h>
#include <random>
#include <fstream>
#include <ros/ros.h>

// Define the LKF we will be using
namespace mrs_lib
{
  const int n_states = 2;
  const int n_inputs = 1;
  const int n_measurements = 1;

  using lkf_t = dtMatrixLKF<n_states, n_inputs, n_measurements>;
  using rep_t = Repredictor<lkf_t>;
}

// Some helpful aliases to make writing of types shorter
using namespace mrs_lib;
using A_t = lkf_t::A_t;
using B_t = lkf_t::B_t;
using H_t = lkf_t::H_t;
using Q_t = lkf_t::Q_t;
using x_t = lkf_t::x_t;
using P_t = lkf_t::P_t;
using u_t = lkf_t::u_t;
using z_t = lkf_t::z_t;
using R_t = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

// Some helper enums to make the code more readable
enum x_pos
{
  x_x = 0,
  x_dx = 2,
};
enum u_pos
{
  u_dx = 0,
};
enum z_pos
{
  z_x = 0,
};

static std::random_device rd{};
static std::mt19937 gen{rd()};
static std::normal_distribution<> d{0,1};

// Helper function to generate a random Eigen matrix with normal distribution
template <int rows>
Eigen::Matrix<double, rows, 1> normal_randmat(const Eigen::Matrix<double, rows, rows>& cov)
{
    Eigen::Matrix<double, rows, 1> ret;
    for (int row = 0; row < rows; row++)
      ret(row, 0) = d(gen);
    return cov*ret;
}

A_t generateA(const double dt)
{
  A_t A;
  A << 1, dt,
       0, 1;
  return A;
}

B_t generateB([[maybe_unused]] const double dt)
{
  B_t B;
  B << dt*dt/2.0,
       dt;
  return B;
}

int main()
{
  // Generate initial state, input and time
  const x_t x0 = x_t::Zero();
  const P_t P0 = 10.0*P_t::Identity();
  const u_t u0 = u_t::Random();
  const ros::Time t0 = ros::Time(0);
  // H will observe the position
  const H_t H( (H_t() << 1, 0).finished() );
  // process noise is just identity
  const Q_t Q = 0.1*Q_t::Identity();
  // measurement noise is just identity
  const R_t R = 0.1*R_t::Identity();
  
  const int n_gts = 5e1;
  const int n_meass = 3e1;

  // Instantiate the LKF model
  auto lkf = std::make_shared<lkf_t>(generateA, generateB, H);

  /* Prepare the ground-truth states //{ */
  
  std::vector<x_t> gts(n_gts);
  std::vector<u_t> inputs(n_gts);
  std::vector<ros::Time> stamps(n_gts);
  
  gts.front() = x0;
  inputs.front() = u0;
  stamps.front() = t0;
  
  for (int it = 1; it < n_gts; it++)
  {
    // Generate a random dt
    const double dt = std::abs(d(gen));
    // Scale Q accordingly
    const Q_t cur_Q = dt*Q;
    // Generate a new state according to the model and noise parameters
    gts.at(it) = generateA(dt)*gts.at(it-1) + generateB(dt)*inputs.at(it-1) + normal_randmat(cur_Q);
    // Add the corresponding stamp
    stamps.at(it) = stamps.at(it-1) + ros::Duration(dt);
    // Generate a new input vector
    inputs.at(it) = u_t::Random();
  }
  
  //}

  /* Generate measurements //{ */
  
  ros::Time prev_stamp = t0;
  int prev_gt_it = 0;
  std::vector<std::tuple<ros::Time, z_t, x_t, u_t>> measurements(n_meass);
  std::vector<statecov_t> lkf_scs(n_meass);
  lkf_scs.front() = {x0, P0};
  statecov_t lkf_sc;
  ros::Time last_lkf_stamp = stamps.front();
  for (int it = 0; it < n_meass; it++)
  {
    // Generate a random dt
    const ros::Time meas_stamp = prev_stamp + ros::Duration(std::abs(d(gen))/n_meass*n_gts);
    // Move the prev_gt_it if necessary
    while (prev_gt_it < n_gts && stamps.at(prev_gt_it) < meas_stamp)
    {
      if (prev_gt_it > 0)
      {
        const auto u = inputs.at(prev_gt_it-1);
        const auto stamp = stamps.at(prev_gt_it);
        const double dt = (stamp-last_lkf_stamp).toSec();
        lkf_sc = lkf->predict(lkf_sc, u, Q, dt);
        last_lkf_stamp = stamp;
      }
      prev_gt_it++;
    }
    prev_gt_it--;
    // Get the closest last ground truths
    const auto x = gts.at(prev_gt_it);
    const auto u = inputs.at(prev_gt_it);
    const auto gt_stamp = stamps.at(prev_gt_it);
    // Calculate dt from last ground truth to measurement stamp
    const double dt = (meas_stamp - gt_stamp).toSec();
    // Calculate the ground truth state at the measurement stamp
    const x_t x_cur = generateA(dt)*x + generateB(dt)*u;
    // Generate a new observation according to the model and noise parameters
    const z_t z = H*x_cur + normal_randmat(R);
    // Add the new measurement and its stamp to the vector
    measurements.at(it) = {meas_stamp, z, x_cur, u};

    lkf_sc = lkf->predict(lkf_sc, u, Q, (meas_stamp-last_lkf_stamp).toSec());
    lkf_sc = lkf->correct(lkf_sc, z, R);
    lkf_scs.at(it) = lkf_sc;

    // Update the helper variable
    prev_stamp = meas_stamp;
  }

  //}
  
  // Instantiate the Repredictor itself
  rep_t rep(x0, P0, u0, Q, t0, lkf, n_gts+n_meass);

  // Fill the buffer of the Repredictor
  auto meas_remaining = measurements;
  int u_it = 1; // the first input is already used for initialization, skip it
  for (int it = 1; it < n_meass+n_inputs; it++)
  {
    const bool use_meas = (d(gen) > 0.0 || u_it == n_gts) && !meas_remaining.empty();
    if (use_meas)
    {
      // add the measurements randomly
      std::uniform_int_distribution<> ud(0, meas_remaining.size()-1);
      const int meas_idx = ud(gen);
      const ros::Time stamp = std::get<0>(meas_remaining.at(meas_idx));
      const z_t z = std::get<1>(meas_remaining.at(meas_idx));
      meas_remaining.erase(std::begin(meas_remaining) + meas_idx);
      rep.addMeasurement(z, R, stamp);
    }
    else
    {
      const u_t u = inputs.at(u_it);
      const ros::Time stamp = stamps.at(u_it);
      rep.addInput(u, Q, stamp);
      u_it++;
    }
  }
  
  {
    std::ofstream ofs("inpt.csv");
    ofs << "t,xgt,dxgt,xes,dxes,xkf,dxkf,err" << std::endl;
    for (int it = 0; it < n_meass; it++)
    {
      const auto [stamp, z, x_gt, u] = measurements.at(it);
      const auto [x, P] = rep.predictTo(stamp);
      const auto lkf_sc = lkf_scs.at(it);
      const auto err = (x_gt-x).norm();
      std::cout << "xgt[" << it << "]: [" << x_gt.transpose() << "]^T\txes[" << it << "]: [" << x.transpose() << "]^T" << "\terr[" << it << "]:  " << err << std::endl;
      ofs << stamp.toSec() << "," << x_gt.x() << "," << x_gt.y() << "," << x.x() << "," << x.y() << "," << lkf_sc.x.x() << "," << lkf_sc.x.y() << "," << err << std::endl;
    }
  }

  {
    std::ofstream ofs("meas.csv");
    ofs << "t,xmeas,xgt,dxgt" << std::endl;
    for (const auto& el : measurements)
    {
      ofs << std::get<0>(el).toSec() << "," << std::get<1>(el) << "," << std::get<2>(el).x() << "," << std::get<2>(el).y() << std::endl;
    }
  }


  return 0;
}


