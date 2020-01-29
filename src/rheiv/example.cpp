// clang: MatousFormat
/**  \file
     \brief Example file for the RHEIV implementation
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib rheiv_example`.

     A plane surface model in the cartesian form \f$ a x + b y + c z + d = 0 \f$ will be used in this example.
     The parameters \f$ a \f$, \f$ b \f$, \f$ c \f$ and \f$ d \f$ are elements of the parameter vector \f$ \mathbf{ \theta } \f$.

     See \ref rheiv/example.cpp.
 */

/**  \example "rheiv/example.cpp"

     This example may be run after building *mrs_lib* by executing `rosrun mrs_lib rheiv_example`.

     A plane surface model in the cartesian form \f$ a x + b y + c z + d = 0 \f$ will be used in this example.
     The parameters \f$ a \f$, \f$ b \f$, \f$ c \f$ and \f$ d \f$ are elements of the parameter vector \f$ \mathbf{ \theta } \f$.
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
  using xs_t = rheiv_t::xs_t;
  using zs_t = rheiv_t::zs_t;
  using P_t = rheiv_t::P_t;
  using Ps_t = rheiv_t::Ps_t;
  using f_z_t = rheiv_t::f_z_t;
  using dzdx_t = rheiv_t::dzdx_t;
}

/* For the plane surface model, there is no need to transform the data. */
mrs_lib::zs_t f_z(const mrs_lib::xs_t& xs)
{
  return xs;
}

int main()
{
  /* For the plane surface model, the Jacobian is just an identitiy matrix. */
  const mrs_lib::dzdx_t dzdx = mrs_lib::dzdx_t::Identity();

  const int n_pts = 4;
  mrs_lib::xs_t xs(3, n_pts);
  xs <<
    0, 1, 0, 10,
    0, 0, 1, 0,
    0, 0, 0, 0.001;  // if you change the last value to 0, the iterative optimization will fail because the matrices M and N will be degenerate
  const mrs_lib::P_t P = mrs_lib::P_t::Identity();
  mrs_lib::Ps_t Ps;
  Ps.reserve(n_pts);
  for (int it = 0; it < n_pts; it++)
    Ps.push_back(P);

  std::cout << "initializing object:" << std::endl;
  mrs_lib::rheiv_t rheiv(f_z, dzdx, 1e-15, 10, std::chrono::milliseconds(1), 1);
  std::cout << "running fit" << std::endl;
  mrs_lib::theta_t theta;
  try
  {
    theta = rheiv.fit(xs, Ps);
  } catch (const mrs_lib::eigenvector_exception& ex)
  {
    std::cerr << "Iteration failed with message '" << ex.what() << "'." << std::endl;
    std::cout << "using last valid theta" << std::endl;
    theta = rheiv.get_last_estimate();
  }
  std::cout << "finished:" << std::endl << theta << std::endl;
}


