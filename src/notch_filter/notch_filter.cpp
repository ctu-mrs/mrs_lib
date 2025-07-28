#include <mrs_lib/notch_filter.h>

namespace mrs_lib
{

// | ------------------------ NotchFilter ------------------------ |

/* NotchFilter constructor //{ */

NotchFilter::NotchFilter(const double& sample_rate, const double& f_in, const double& b_in) {

  double sf  = sample_rate;
  double sf2 = sf / 2;

  std::vector<double> frequencies_in;
  frequencies_in.push_back(f_in);
  std::vector<double> bandwidths_in;
  bandwidths_in.push_back(b_in);

  if (frequencies_in.size() != bandwidths_in.size() || frequencies_in.size() == 0) {
    throw std::runtime_error("Parameters frequencies and bandwidths needs to have the same non-zero length!!");
  }

  size_t order = frequencies_in.size();

  /* std::vector<double>   frequencies; */
  Eigen::VectorXd frequencies(order);
  Eigen::VectorXd bandwidths(order);

  for (size_t i = 0; i < order; i++) {
    frequencies[i] = frequencies_in[i] / sf2;
    bandwidths[i]  = (bandwidths_in[i] / sf2);
  }

  for (size_t i = 0; i < order; i++) {
    frequencies[i] *= M_PI;
    bandwidths[i] *= M_PI;
  }

  Eigen::VectorXd omega(order * 2);

  for (size_t i = 0; i < order; i++) {
    omega[2 * i]     = (frequencies[i] - (bandwidths[i] / 2));
    omega[2 * i + 1] = (frequencies[i]);
  }

  Eigen::VectorXi factors(order);
  int             factor = 1;

  for (size_t i = 0; i < order; i++) {
    factors(i) = factor;
    factor += 2;
  }

  Eigen::VectorXd phi(order * 2);

  for (size_t i = 0; i < order; i++) {
    phi[2 * i]     = (-M_PI * factors[i] + (M_PI / 2));
    phi[2 * i + 1] = (-M_PI * factors[i]);
  }

  Eigen::VectorXd t_beta(order * 2);

  for (size_t i = 0; i < order * 2; i++) {
    t_beta[i] = (tan((phi[i] + (2 * order) * omega[i]) / 2));
  }

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(order * 2, order * 2);

  for (size_t i = 0; i < (order * 2); i++) {
    int k    = i + 1;
    Q.col(i) = (k * omega).array().sin() - (t_beta.array() * ((k * omega).array().cos()).array());
  }

  Eigen::VectorXd h_a = (Q.transpose() * Q).inverse() * (Q.transpose()) * t_beta;

  Eigen::VectorXd nom(h_a.size() + 1);
  Eigen::VectorXd denom(h_a.size() + 1);

  Eigen::VectorXd h_a_rev = h_a.reverse();

  denom[0]        = (1.0);
  nom[h_a.size()] = (1.0);

  for (long i = 0; i < h_a.size(); i++) {
    denom[i + 1] = h_a[i];
    nom[i]       = h_a_rev[i];
  }

  Eigen::VectorXd a = denom;
  Eigen::VectorXd b = (nom + denom) / 2;

  std::vector<double> a_std;
  std::vector<double> b_std;

  for (long i = 0; i < a.size(); i++) {
    a_std.push_back(a[i]);
    b_std.push_back(b[i]);
  }

  filter = std::make_unique<mrs_lib::IirFilter>(a_std, b_std);
  std::cout << "Notch filter initialized!" << std::endl;
}

//}

/* iterate //{ */

double NotchFilter::iterate(double& sample_in) {
  return filter.get()->iterate(sample_in);
}

//}

//}

}  // namespace mrs_lib
