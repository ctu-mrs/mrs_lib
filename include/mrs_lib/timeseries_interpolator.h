/**
 * @file TimeseriesInterpolator.h
 * @brief A simple class to allow interpolation for trajectories that are fed
 * into the class using std::vectors
 *
 * @author Parakh M. Gupta
 */

#include <iostream>
#include <vector>

namespace mrs_lib {

/**
 * @brief A simple class to allow interpolation for trajectories
 *
 */

class TimeseriesInterpolator {
public:
  /**
   * @brief Interpolates the timeseries data using linear interpolation by
   * finding the corresponding time in a series that lies between two entries
   *
   * @param input_timestamps The time vector of the current timeseries, expected
   * to be monotonically increasing
   * @param data The data vector where every element will be interpolated
   * @param desired_timestamps The timestamps at which the data will be
   * interpolated, expected to be monotonically increasing
   *
   * @return The interpolated timeseries, returns empty vector when conditions
   * not met
   */
  std::vector<std::vector<double>>
  getInterpolatedTimeseries(const std::vector<double> &input_timestamps,
                            const std::vector<std::vector<double>> &data,
                            const std::vector<double> &desired_timestamps);

private:
  /**
   * @brief Linear interpolation function
   *
   * @param a The first value
   * @param b The second value
   * @param t The fraction of the distance between a and b
   *
   * @return The interpolated value
   */
  double lerp(double a, double b, double t) { return a + t * (b - a); };
};

} // namespace mrs_lib
