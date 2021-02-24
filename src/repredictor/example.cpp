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
#include <mrs_lib/mutex.h>
#include <random>
#include <fstream>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <mrs_msgs/Float64Stamped.h>

// Define the LKF we will be using
namespace mrs_lib
{
  const int n_states = 2;
  const int n_inputs = 1;
  const int n_measurements = 1;

  using lkf_t = varstepLKF<n_states, n_inputs, n_measurements>;
  using rep_t = Repredictor<lkf_t>;
}

/* helper aliases and definitions //{ */

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
static std::normal_distribution<> d{0,1};

// Helper function to generate a random Eigen matrix with normal distribution
template <int rows>
Eigen::Matrix<double, rows, 1> multivariate_gaussian(const Eigen::Matrix<double, rows, rows>& cov)
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

const Q_t Q = 2.5*Q_t::Identity();
const H_t H( (H_t() << 1, 0).finished() );
const R_t R_fast = 5.5*R_t::Identity();
const R_t R_slow = 0.01*R_t::Identity();

const x_t x0 = x_t::Zero();
const P_t P0 = 5.0*P_t::Identity();
const u_t u0 = u_t::Zero();
const ros::Time t0 = ros::Time(0);
const std::shared_ptr<lkf_t> lkf_ptr = std::make_shared<lkf_t>(generateA, generateB, H);
const unsigned buf_sz = 100;

std::mutex rep_pubs_mtx;
rep_t rep(x0, P0, u0, Q, t0, lkf_ptr, buf_sz);
ros::Publisher pub_pos_est;
ros::Publisher pub_vel_est;

enum type_e
{
  input,
  meas_fast,
  meas_slow
};

//}

// --------------------------------------------------------------
// |                     THE IMPORTANT PART                     |
// --------------------------------------------------------------
void process_msg(const mrs_msgs::Float64Stamped::ConstPtr msg, type_e type)
{
  // multithreading synchronization
  std::scoped_lock lck(rep_pubs_mtx);

  // update the Repredictor with the latest message based on its type
  switch (type)
  {
    case input: // interpret msg as system input
      rep.addInputChangeWithNoise(u_t(msg->value), Q, msg->header.stamp);
      break;
    case meas_fast: // interpret msg as fast measurement (use the corresponding R)
      rep.addMeasurement(z_t(msg->value), R_fast, msg->header.stamp);
      break;
    case meas_slow: // interpret msg as slow measurement (use the corresponding R)
      rep.addMeasurement(z_t(msg->value), R_slow, msg->header.stamp);
      break;
  }

  // publish the latest estimate
  mrs_msgs::Float64Stamped msg_out;
  msg_out.header.stamp = ros::Time::now();

  // estimate the current state using Repredictor
  const x_t x_pred = rep.predictTo(msg_out.header.stamp).x;

  // publish the position estimate
  msg_out.value = x_pred.x();
  pub_pos_est.publish(msg_out);

  // publish the velocity estimate
  msg_out.value = x_pred.y();
  pub_vel_est.publish(msg_out);
}

/* callback splitting to process_msg() //{ */

void callback_input(const mrs_msgs::Float64Stamped::ConstPtr msg)
{
  process_msg(msg, input);
}

void callback_meas_fast(const mrs_msgs::Float64Stamped::ConstPtr msg)
{
  process_msg(msg, meas_fast);
}

void callback_meas_slow(const mrs_msgs::Float64Stamped::ConstPtr msg)
{
  process_msg(msg, meas_slow);
}

//}

// current state and its mutex
std::mutex x_mtx;
x_t x = x_t::Random();

/* publisher thread of fast, imprecise measurements //{ */

ros::Publisher pub_meas_fast;
void meas_generator_fast()
{
  const R_t R = R_fast;
  const double delay_std = 0.1;

  while (ros::ok())
  {
    const x_t x_gt = mrs_lib::get_mutexed(x_mtx, x);
    const z_t z = H*x_gt + multivariate_gaussian(R);
    const ros::Time stamp = ros::Time::now();
    const double delay = delay_std*std::abs(d(gen));
    ros::Duration(delay).sleep();
    mrs_msgs::Float64Stamped msg;
    msg.header.stamp = stamp;
    msg.value = z.x();
    pub_meas_fast.publish(msg);
  }
}

//}

/* publisher thread of slow, precise measurements //{ */

ros::Publisher pub_meas_slow;
void meas_generator_slow()
{
  const R_t R = R_slow;
  const double delay_std = 1.0;

  while (ros::ok())
  {
    const x_t x_gt = mrs_lib::get_mutexed(x_mtx, x);
    const z_t z = H*x_gt + multivariate_gaussian(R);
    const ros::Time stamp = ros::Time::now();
    const double delay = delay_std*std::abs(d(gen));
    ros::Duration(delay).sleep();
    mrs_msgs::Float64Stamped msg;
    msg.header.stamp = stamp;
    msg.value = z.x();
    pub_meas_slow.publish(msg);
  }
}

//}

int main(int argc, char **argv)
{
  /* Set up ROS. */
  const std::string node_name("repredictor_example");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // subscriber on the topic
  ros::Subscriber sub_inpt = nh.subscribe("velocity_inpt", 10, callback_input);
  ros::Subscriber sub_meas_fast = nh.subscribe("position_meas_fast", 10, callback_meas_fast);
  ros::Subscriber sub_meas_slow = nh.subscribe("position_meas_slow", 10, callback_meas_slow);

  // publishers for publishing generated measurements, ground truths and the estimate
  pub_pos_est = nh.advertise<mrs_msgs::Float64Stamped>("position_est", 5);
  pub_vel_est = nh.advertise<mrs_msgs::Float64Stamped>("velocity_est", 5);
  pub_meas_fast = nh.advertise<mrs_msgs::Float64Stamped>("position_meas_fast", 5);
  pub_meas_slow = nh.advertise<mrs_msgs::Float64Stamped>("position_meas_slow", 5);
  ros::Publisher pub_inpt_gt = nh.advertise<mrs_msgs::Float64Stamped>("velocity_inpt", 5);
  ros::Publisher pub_pos_gt = nh.advertise<mrs_msgs::Float64Stamped>("position_gt", 5);
  ros::Publisher pub_vel_gt = nh.advertise<mrs_msgs::Float64Stamped>("velocity_gt", 5);

  // threads for publishing 
  std::thread th_fast(meas_generator_fast);
  th_fast.detach();
  std::thread th_slow(meas_generator_slow);
  th_slow.detach();

  // our system model, generating the currant state x
  ros::Duration dur(5e-2);
  while (ros::ok())
  {
    const double dt = dur.toSec();
    const u_t u = 3.0*u_t::Random();
    const Q_t dtQ = dt*Q;

    // generate a new state
    const x_t x_old = mrs_lib::get_mutexed(x_mtx, x);
    const x_t x_new = generateA(dt)*x_old + generateB(dt)*u + multivariate_gaussian(dtQ);
    mrs_lib::set_mutexed(x_mtx, x_new, x);

    mrs_msgs::Float64Stamped msg;

    // publish the position ground-truth
    msg.header.stamp = ros::Time::now() - ros::Duration(dt);
    msg.value = u.x();
    pub_inpt_gt.publish(msg);

    // publish the position ground-truth
    msg.header.stamp = ros::Time::now();
    msg.value = x.x();
    pub_pos_gt.publish(msg);

    // publish the velocity ground-truth
    msg.header.stamp = ros::Time::now();
    msg.value = x.y();
    pub_vel_gt.publish(msg);

    ROS_INFO_THROTTLE(1.0, "[%s]: Spinning", ros::this_node::getName().c_str());
    ros::spinOnce();
    dur.sleep();
  }
}


