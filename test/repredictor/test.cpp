#include <gtest/gtest.h>

#include <mrs_lib/repredictor.h>
#include <mrs_lib/lkf.h>

// Include the LKF header
#include <mrs_lib/lkf.h>
#include <random>

// Define the LKF we will be using
namespace mrs_lib
{
  const int n_states = 2;
  const int n_inputs = 1;
  const int n_measurements = 1;

  using lkf_t = varstepLKF<n_states, n_inputs, n_measurements>;
  using rep_t = Repredictor<lkf_t>;
  using dumbrep_t = Repredictor<lkf_t, true>;
}  // namespace mrs_lib

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

static std::random_device rd{};
static std::mt19937 gen{rd()};
static std::normal_distribution<> d{0, 1};

// Helper function to generate a random Eigen matrix with normal distribution
template <int rows>
Eigen::Matrix<double, rows, 1> normal_randmat(const Eigen::Matrix<double, rows, rows>& cov)
{
  Eigen::Matrix<double, rows, 1> ret;
  for (int row = 0; row < rows; row++)
    ret(row, 0) = d(gen);
  return cov * ret;
}

A_t generateA(const double dt)
{
  A_t A;
  A << 1, dt, 0, 1;
  return A;
}

B_t generateB([[maybe_unused]] const double dt)
{
  B_t B;
  B << dt * dt / 2.0, dt;
  return B;
}

/* TEST(TESTSuite, lkf_comparison) //{ */

TEST(TESTSuite, lkf_comparison)
/* int main() */
{
  // Generate initial state, input and time
  const x_t x0 = x_t::Zero();
  const P_t P0 = 10.0 * P_t::Identity();
  const u_t u0 = u_t::Random();
  const rclcpp::Time t0 = rclcpp::Time(0, 0);

  // H will observe the position
  const H_t H((H_t() << 1, 0).finished());

  // process noise is just identity
  const Q_t Q = 0.1 * Q_t::Identity();

  // measurement noise is just identity
  const R_t R = 0.1 * R_t::Identity();

  const int n_gts = 2e2;
  const int n_meass = 1e2;

  // Instantiate the LKF model
  auto lkf = std::make_shared<lkf_t>(generateA, generateB, H);

  /* Prepare the ground-truth states //{ */

  std::vector<x_t> gts(n_gts);
  std::vector<u_t> inputs(n_gts);
  std::vector<rclcpp::Time> stamps(n_gts);

  gts.front() = x0;
  inputs.front() = u0;
  stamps.front() = t0;

  for (int it = 1; it < n_gts; it++)
  {

    // Generate a random dt
    const double dt = std::abs(d(gen));

    // Scale Q accordingly
    const Q_t cur_Q = dt * Q;

    // Generate a new state according to the model and noise parameters
    gts.at(it) = generateA(dt) * gts.at(it - 1) + generateB(dt) * inputs.at(it - 1) + normal_randmat(cur_Q);

    // Add the corresponding stamp
    stamps.at(it) = stamps.at(it - 1) + rclcpp::Duration(std::chrono::duration<double>(dt));

    // Generate a new input vector
    inputs.at(it) = u_t::Random();
  }

  //}

  /* Generate measurements //{ */

  rclcpp::Time prev_stamp = t0;
  int prev_gt_it = 0;

  std::vector<std::tuple<rclcpp::Time, z_t, x_t, u_t>> measurements(n_meass);
  std::vector<statecov_t> lkf_scs(n_meass);
  statecov_t lkf_sc = {x0, P0, t0};
  rclcpp::Time last_lkf_stamp = stamps.front();
  std::cout << "LKF:" << std::endl;

  for (int it = 0; it < n_meass; it++)
  {

    // Generate a random dt
    const rclcpp::Time meas_stamp = prev_stamp + rclcpp::Duration(std::chrono::duration<double>(std::abs(d(gen)) / n_meass * n_gts));

    // Move the prev_gt_it if necessary
    while (prev_gt_it < n_gts && stamps.at(prev_gt_it) < meas_stamp)
    {
      const auto stamp = stamps.at(prev_gt_it);
      if (prev_gt_it > 0 && stamp > last_lkf_stamp)
      {
        const auto u = inputs.at(prev_gt_it - 1);
        const double dt = (stamp - last_lkf_stamp).seconds();
        lkf_sc = lkf->predict(lkf_sc, u, Q, dt);
        std::cout << "predict\t" << rclcpp::Time(last_lkf_stamp).seconds() << "\t->\t" << rclcpp::Time(stamp).seconds() << std::endl;
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
    const double dt = (meas_stamp - gt_stamp).seconds();

    // Calculate the ground truth state at the measurement stamp
    const x_t x_cur = generateA(dt) * x + generateB(dt) * u;

    // Generate a new observation according to the model and noise parameters
    const z_t z = H * x_cur + normal_randmat(R);

    // Add the new measurement and its stamp to the vector
    measurements.at(it) = {meas_stamp, z, x_cur, u};

    lkf_sc = lkf->predict(lkf_sc, u, Q, (meas_stamp - last_lkf_stamp).seconds());
    std::cout << "predict\t" << rclcpp::Time(last_lkf_stamp).seconds() << "\t->\t" << rclcpp::Time(meas_stamp).seconds() << std::endl;
    last_lkf_stamp = meas_stamp;
    lkf_sc = lkf->correct(lkf_sc, z, R);
    std::cout << "correct\t" << rclcpp::Time(meas_stamp).seconds() << std::endl;
    lkf_scs.at(it) = lkf_sc;

    // Update the helper variable
    prev_stamp = meas_stamp;
  }

  rclcpp::Time tend = stamps.back() > std::get<0>(measurements.back()) ? stamps.back() : std::get<0>(measurements.back());
  tend += rclcpp::Duration(std::chrono::duration<double>(0.2));
  prev_gt_it++;

  while (prev_gt_it < (int)stamps.size())
  {

    const auto u = inputs.at(prev_gt_it - 1);
    const auto stamp = stamps.at(prev_gt_it);
    const double dt = (stamp - last_lkf_stamp).seconds();

    lkf_sc = lkf->predict(lkf_sc, u, Q, dt);
    std::cout << "predict\t" << rclcpp::Time(last_lkf_stamp).seconds() << "\t->\t" << rclcpp::Time(stamp).seconds() << std::endl;
    last_lkf_stamp = stamp;

    prev_gt_it++;
  }

  const auto u = inputs.at(prev_gt_it - 1);
  const auto stamp = tend;
  const double dt = (stamp - last_lkf_stamp).seconds();
  lkf_sc = lkf->predict(lkf_sc, u, Q, dt);
  std::cout << "predict\t" << rclcpp::Time(last_lkf_stamp).seconds() << "\t->\t" << rclcpp::Time(stamp).seconds() << std::endl;
  last_lkf_stamp = tend;

  //}

  // Instantiate the Repredictor itself
  rep_t rep(x0, P0, u0, Q, t0, lkf, n_gts + n_meass);

  // Fill the buffer of the Repredictor
  auto meas_remaining = measurements;
  int u_it = 1;  // the first input is already used for initialization, skip it
  for (int it = 1; it < n_meass + n_gts; it++)
  {
    const bool use_meas = (d(gen) > 0.0 || u_it == n_gts) && !meas_remaining.empty();
    if (use_meas)
    {
      // add the measurements randomly
      std::uniform_int_distribution<> ud(0, meas_remaining.size() - 1);
      const int meas_idx = ud(gen);
      const rclcpp::Time stamp = std::get<0>(meas_remaining.at(meas_idx));
      const z_t z = std::get<1>(meas_remaining.at(meas_idx));
      meas_remaining.erase(std::begin(meas_remaining) + meas_idx);
      rep.addMeasurement(z, R, stamp);
    } else
    {
      const u_t u = inputs.at(u_it);
      const rclcpp::Time stamp = stamps.at(u_it);
      rep.addInputChangeWithNoise(u, Q, stamp);
      u_it++;
    }
  }

  {
    /* std::ofstream ofs("inpt.csv"); */
    /* ofs << "t,xgt,dxgt,xes,dxes,xkf,dxkf,err,diffx,diffP" << std::endl; */
    auto rep_sc = rep.predictTo(t0);
    /* ofs << t0.toSec() << "," << x0.x() << "," << x0.y() << "," << rep_sc.x.x() << "," << rep_sc.x.y() << "," << x0.x() << "," << x0.y() << "," <<
     * (rep_sc.x-x0).norm() << std::endl; */
    for (int it = 0; it < n_meass; it++)
    {
      const auto stamp = std::get<0>(measurements.at(it));
      /* const auto x_gt = std::get<2>(measurements.at(it)); */
      rep_sc = rep.predictTo(stamp);
      const auto lkf_sc = lkf_scs.at(it);
      /* const auto err = (x_gt-rep_sc.x).norm(); */
      const auto diff = (lkf_sc.x - rep_sc.x).norm();
      const auto diffP = (lkf_sc.P - rep_sc.P).norm();
      EXPECT_DOUBLE_EQ(diff, 0.0);
      EXPECT_DOUBLE_EQ(diffP, 0.0);
      /* std::cout << "xgt[" << it << "]: [" << x_gt.transpose() << "]^T\txes[" << it << "]: [" << rep_sc.x.transpose() << "]^T" << "\terr[" << it << "]:  " <<
       * err << std::endl; */
      /* ofs << stamp.toSec() << "," << x_gt.x() << "," << x_gt.y() << "," << rep_sc.x.x() << "," << rep_sc.x.y() << "," << lkf_sc.x.x() << "," << lkf_sc.x.y()
       * << "," << err << "," << diff << "," << diffP << std::endl; */
    }
    rep_sc = rep.predictTo(tend);
    /* const x_t x_gt = generateA((tend-stamps.back()).toSec())*gts.back() + generateB((tend-stamps.back()).toSec())*inputs.back(); */
    /* const auto err = (x_gt-rep_sc.x).norm(); */
    const auto diff = (lkf_sc.x - rep_sc.x).norm();
    const auto diffP = (lkf_sc.P - rep_sc.P).norm();
    EXPECT_DOUBLE_EQ(diff, 0.0);
    EXPECT_DOUBLE_EQ(diffP, 0.0);
    /* ofs << stamp.toSec() << "," << x_gt.x() << "," << x_gt.y() << "," << rep_sc.x.x() << "," << rep_sc.x.y() << "," << lkf_sc.x.x() << "," << lkf_sc.x.y() <<
     * "," << err << "," << diff << "," << diffP << std::endl; */
  }

  /* { */
  /*   std::ofstream ofs("meas.csv"); */
  /*   ofs << "t,xmeas,xgt,dxgt" << std::endl; */
  /*   for (const auto& el : measurements) */
  /*   { */
  /*     ofs << std::get<0>(el).toSec() << "," << std::get<1>(el) << "," << std::get<2>(el).x() << "," << std::get<2>(el).y() << std::endl; */
  /*   } */
  /* } */
  /* return 0; */
}

//}

/* TEST(TESTSuite, dumblkf_comparison) //{ */

TEST(TESTSuite, dumblkf_comparison)
/* int main() */
{
  // Generate initial state, input and time
  const x_t x0 = x_t::Zero();
  const P_t P0 = 10.0 * P_t::Identity();
  const u_t u0 = u_t::Random();
  const rclcpp::Time t0 = rclcpp::Time(0, 0);

  // H will observe the position
  const H_t H((H_t() << 1, 0).finished());

  // process noise is just identity
  const Q_t Q = 0.1 * Q_t::Identity();

  // measurement noise is just identity
  const R_t R = 0.1 * R_t::Identity();

  const int n_gts = 2e2;

  // Instantiate the LKF model
  auto lkf = std::make_shared<lkf_t>(generateA, generateB, H);

  std::cout << "Preparing measurements." << std::endl;
  /* Prepare the ground-truth states and measurements //{ */

  std::vector<x_t> gts(n_gts);
  std::vector<u_t> inputs(n_gts);
  std::vector<rclcpp::Time> stamps(n_gts);
  std::vector<z_t> measurements(n_gts);

  gts.front() = x0;
  inputs.front() = u0;
  stamps.front() = t0;
  measurements.front() = H * x0 + normal_randmat(R);

  for (int it = 1; it < n_gts; it++)
  {
    // Generate a random dt
    const double dt = std::abs(d(gen));
    // Scale Q accordingly
    const Q_t cur_Q = dt * Q;
    // Generate a new state according to the model and noise parameters
    gts.at(it) = generateA(dt) * gts.at(it - 1) + generateB(dt) * inputs.at(it - 1) + normal_randmat(cur_Q);
    // Add the corresponding stamp
    stamps.at(it) = stamps.at(it - 1) + rclcpp::Duration(std::chrono::duration<double>(dt));
    // Generate a new input vector
    inputs.at(it) = u_t::Random();
    // Generate a corresponding measurement
    measurements.at(it) = H * gts.at(it) + normal_randmat(R);
  }

  //}

  std::vector<statecov_t> lkf_scs(n_gts);
  std::vector<statecov_t> rep_scs(n_gts);
  statecov_t lkf_sc = {x0, P0, t0};
  u_t u = u0;

  // Instantiate the Repredictor itself
  dumbrep_t rep(x0, P0, u0, Q, t0, lkf, 1);

  std::cout << "Running LKF and dumb Repredictor." << std::endl;

  // Run the LKF and dumb repredictor
  auto meas_remaining = measurements;
  int u_it = 1;  // the first input is already used for initialization, skip it
  for (int it = 1; it < n_gts; it++)
  {

    const bool use_meas = (d(gen) > 0.0 || u_it == n_gts) && !meas_remaining.empty();

    if (use_meas)
    {

      // add the measurements randomly
      std::uniform_int_distribution<> ud(0, meas_remaining.size() - 1);
      const int meas_idx = ud(gen);
      const rclcpp::Time stamp = stamps.at(meas_idx);
      const double dt = (stamp - lkf_sc.stamp).seconds();
      const z_t z = meas_remaining.at(meas_idx);

      meas_remaining.erase(std::begin(meas_remaining) + meas_idx);

      const auto old_stamp = lkf_sc.stamp;

      if (dt > 0)
        lkf_sc = lkf->predict(lkf_sc, u, Q, dt);

      lkf_sc = lkf->correct(lkf_sc, z, R);

      if (dt > 0)
        lkf_sc.stamp = stamp;
      else
        lkf_sc.stamp = old_stamp;

      lkf_scs.at(it) = lkf_sc;

      rep.addMeasurement(z, R, stamp);
      const auto rep_sc = rep.predictTo(lkf_sc.stamp);
      rep_scs.at(it) = rep_sc;

    } else
    {
      u = inputs.at(u_it);
      rep.addInputChangeWithNoise(u, Q, lkf_sc.stamp);
      u_it++;
    }
  }

  std::cout << "Evaluating results." << std::endl;

  for (int it = 0; it < n_gts; it++)
  {

    /* const auto x_gt = gts.at(it); */
    const auto rep_sc = rep_scs.at(it);
    const auto lkf_sc = lkf_scs.at(it);

    /* const auto err = (x_gt-rep_sc.x).norm(); */
    const auto diff = (lkf_sc.x - rep_sc.x).norm();
    const auto diffP = (lkf_sc.P - rep_sc.P).norm();

    EXPECT_DOUBLE_EQ(diff, 0.0);
    EXPECT_DOUBLE_EQ(diffP, 0.0);
    /* std::cout << "xgt[" << it << "]: [" << x_gt.transpose() << "]^T\txes[" << it << "]: [" << rep_sc.x.transpose() << "]^T" << "\terr[" << it << "]:  " <<
     * err << std::endl; */
    /* ofs << stamp.toSec() << "," << x_gt.x() << "," << x_gt.y() << "," << rep_sc.x.x() << "," << rep_sc.x.y() << "," << lkf_sc.x.x() << "," << lkf_sc.x.y() <<
     * "," << err << "," << diff << "," << diffP << std::endl; */
  }

  /* { */
  /*   std::ofstream ofs("meas.csv"); */
  /*   ofs << "t,xmeas,xgt,dxgt" << std::endl; */
  /*   for (const auto& el : measurements) */
  /*   { */
  /*     ofs << std::get<0>(el).toSec() << "," << std::get<1>(el) << "," << std::get<2>(el).x() << "," << std::get<2>(el).y() << std::endl; */
  /*   } */
  /* } */
  /* return 0; */
}

//}

/* TEST(TESTSuite, dumblkf_comparison2) //{ */

TEST(TESTSuite, dumblkf_comparison2)
/* int main() */
{
  // Generate initial state, input and time
  const x_t x0 = x_t::Zero();
  const P_t P0 = 10.0 * P_t::Identity();
  const u_t u0 = u_t::Random();

  const rclcpp::Time t0 = rclcpp::Time(0);

  // H will observe the position
  const H_t H((H_t() << 1, 0).finished());

  // process noise is just identity
  const Q_t Q = 0.1 * Q_t::Identity();

  // measurement noise is just identity
  const R_t R = 0.1 * R_t::Identity();

  const int n_gts = 2e2;

  // Instantiate the LKF model
  auto lkf = std::make_shared<lkf_t>(generateA, generateB, H);

  std::cout << "Preparing measurements." << std::endl;
  /* Prepare the ground-truth states and measurements //{ */

  std::vector<x_t> gts(n_gts);
  std::vector<u_t> inputs(n_gts);
  std::vector<rclcpp::Time> stamps(n_gts);
  std::vector<z_t> measurements(n_gts);

  gts.front() = x0;
  inputs.front() = u0;
  stamps.front() = t0;
  measurements.front() = H * x0 + normal_randmat(R);

  for (int it = 1; it < n_gts; it++)
  {

    // Generate a random dt
    const double dt = std::abs(d(gen));

    // Scale Q accordingly
    const Q_t cur_Q = dt * Q;

    // Generate a new state according to the model and noise parameters
    gts.at(it) = generateA(dt) * gts.at(it - 1) + generateB(dt) * inputs.at(it - 1) + normal_randmat(cur_Q);

    // Add the corresponding stamp
    stamps.at(it) = stamps.at(it - 1) + rclcpp::Duration(std::chrono::duration<double>(dt));

    // Generate a new input vector
    inputs.at(it) = u_t::Random();

    // Generate a corresponding measurement
    measurements.at(it) = H * gts.at(it) + normal_randmat(R);
  }

  //}

  std::vector<statecov_t> lkf_scs(n_gts);
  std::vector<statecov_t> rep_scs(n_gts);
  statecov_t lkf_sc = {x0, P0, t0};

  // Instantiate the Repredictor itself
  dumbrep_t rep(x0, P0, u0, Q, t0, lkf, 1);

  std::cout << "Running LKF and dumb Repredictor." << std::endl;
  // Run the LKF and dumb repredictor
  for (int it = 1; it < n_gts; it++)
  {

    const bool use_process_noise = d(gen) > 0.0;

    // add the measurements randomly
    const rclcpp::Time stamp = stamps.at(it);
    const double dt = (stamp - lkf_sc.stamp).seconds();
    const z_t& z = measurements.at(it);
    const u_t& u = inputs.at(it);

    if (use_process_noise)
      rep.addInputChangeWithNoise(u, Q, lkf_sc.stamp);
    else
      rep.addInputChange(u, lkf_sc.stamp);

    rep.addMeasurement(z, R, stamp);
    auto rep_sc = rep.predictTo(stamp);
    rep_sc.stamp = stamp;
    rep_scs.at(it) = rep_sc;

    lkf_sc = lkf->predict(lkf_sc, u, Q, dt);
    lkf_sc = lkf->correct(lkf_sc, z, R);
    lkf_sc.stamp = stamp;
    lkf_scs.at(it) = lkf_sc;
  }

  std::cout << "Evaluating results." << std::endl;
  for (int it = 0; it < n_gts; it++)
  {
    /* const auto x_gt = gts.at(it); */
    const auto rep_sc = rep_scs.at(it);
    const auto lkf_sc = lkf_scs.at(it);
    /* const auto err = (x_gt-rep_sc.x).norm(); */
    const auto diff = (lkf_sc.x - rep_sc.x).norm();
    const auto diffP = (lkf_sc.P - rep_sc.P).norm();
    EXPECT_DOUBLE_EQ(diff, 0.0);
    EXPECT_DOUBLE_EQ(diffP, 0.0);
    /* std::cout << "xgt[" << it << "]: [" << x_gt.transpose() << "]^T\txes[" << it << "]: [" << rep_sc.x.transpose() << "]^T" << "\terr[" << it << "]:  " <<
     * err << std::endl; */
    /* ofs << stamp.toSec() << "," << x_gt.x() << "," << x_gt.y() << "," << rep_sc.x.x() << "," << rep_sc.x.y() << "," << lkf_sc.x.x() << "," << lkf_sc.x.y() <<
     * "," << err << "," << diff << "," << diffP << std::endl; */
  }

  /* { */
  /*   std::ofstream ofs("meas.csv"); */
  /*   ofs << "t,xmeas,xgt,dxgt" << std::endl; */
  /*   for (const auto& el : measurements) */
  /*   { */
  /*     ofs << std::get<0>(el).toSec() << "," << std::get<1>(el) << "," << std::get<2>(el).x() << "," << std::get<2>(el).y() << std::endl; */
  /*   } */
  /* } */
  /* return 0; */
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
