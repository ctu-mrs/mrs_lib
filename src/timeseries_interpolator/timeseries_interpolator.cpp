
#include <mrs_lib/timeseries_interpolator.h>

namespace mrs_lib {

// getInterpolatedTimeseries
std::vector<std::vector<double>>
TimeseriesInterpolator::getInterpolatedTimeseries(const std::vector<double> &input_timestamps,
                                                  const std::vector<std::vector<double>> &data,
                                                  const std::vector<double> &desired_timestamps) {
  if (input_timestamps.size() < 2 || data.size() < 2) {
    std::cout << "TimeseriesInterpolator: input_timestamps and input_data must have at least 2 elements, empty vector"
                 "returned\n";
    return std::vector<std::vector<double>>();
  }
  if (desired_timestamps.size() < 1) {
    std::cout << "TimeseriesInterpolator: desired_timestamps must have at least 1 element, empty vector returned\n";
    return std::vector<std::vector<double>>();
  }
  if (input_timestamps.size() != data.size()) {
    std::cout << "TimeseriesInterpolator: input_timestamps and data don't have the same size, interpolation will "
                 "return the smaller of the two\n";
  }
  if (desired_timestamps.at(0) > input_timestamps.at(input_timestamps.size() - 1) ||
      desired_timestamps.at(desired_timestamps.size() - 1) < input_timestamps.at(0)) {
    std::cout << "TimeseriesInterpolator: desired_timestamps don't overlap with input_timestamps, will interpolate but "
                 "verify inputs\n";
  }
  size_t max_search_index = std::min(input_timestamps.size(), data.size());
  size_t search_index = 1;
  size_t desired_time_index = 0;
  std::vector<std::vector<double>> output_timeseries;
  while (desired_time_index < desired_timestamps.size()) {
    while (desired_timestamps.at(desired_time_index) > input_timestamps.at(search_index) &&
           search_index < max_search_index - 1) {
      search_index++;
    }
    std::vector<double> interpolated_data;
    for (size_t i = 0; i < data.at(search_index).size(); i++) {
      double time_fraction = (desired_timestamps.at(desired_time_index) - input_timestamps.at(search_index - 1)) /
                             (input_timestamps.at(search_index) - input_timestamps.at(search_index - 1));
      double interpolated_value = lerp(data.at(search_index - 1).at(i), data.at(search_index).at(i), time_fraction);
      interpolated_data.push_back(interpolated_value);
    }
    output_timeseries.push_back(interpolated_data);
    desired_time_index++;
  }
  return output_timeseries;
}
} // namespace mrs_lib
