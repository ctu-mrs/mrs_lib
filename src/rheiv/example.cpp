// clang: MatousFormat

/**  \file
     \brief Example file for the LKF implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz
     This example may be run after building *mrs_lib* by executing 'rosrun mrs_lib lkf_example'.
 */

// Include the LKF header
#include <mrs_lib/rheiv.h>
#include <ros/ros.h>

// Define the LKF we will be using
namespace mrs_lib
{
  const int n_states = 3;
  const int n_params = 4;

  using rheiv_t = RHEIV<n_states, n_params>;
  using theta_t = rheiv_t::theta_t;
  using us_t = rheiv_t::us_t;
  using P_t = rheiv_t::P_t;
  using Ps_t = rheiv_t::Ps_t;
  using dzdx_t = rheiv_t::dzdx_t;
}


int main()
{
  const mrs_lib::dzdx_t dzdx = mrs_lib::dzdx_t::Identity();

  const int n_pts = 4;
  mrs_lib::us_t us(4, n_pts);
  us <<
    0, 1, 0, 10,
    0, 0, 1, 0,
    0, 0, 0, 0,
    1, 1, 1, 1;
  const mrs_lib::P_t P = mrs_lib::P_t::Identity();
  mrs_lib::Ps_t Ps;
  Ps.reserve(n_pts);
  for (int it = 0; it < n_pts; it++)
    Ps.push_back(P);

  std::cout << "initializing object:" << std::endl;
  mrs_lib::rheiv_t rheiv(dzdx, 1e-15, 100);
  std::cout << "running fit" << std::endl;
  mrs_lib::theta_t theta = rheiv.fit(us, Ps);
  std::cout << "finished:" << std::endl << theta << std::endl;
}


