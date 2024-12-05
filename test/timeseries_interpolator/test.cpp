#include <mrs_lib/timeseries_interpolator.h>
#include <gtest/gtest.h>
#include <vector>

using namespace mrs_lib;



TEST(TESTSuite, trajectory_too_old) {

  int result = 1;

  printf("Testing for trajectory being too old\n");

  std::vector<double> input_timestamps = {0, 1, 2};
  std::vector<double> desired_timestamps = {6, 7, 8, 9, 10};
  std::vector<std::vector<double>> data = {{0, 1, 2, 3, 4, 5}, {1, 2, 3, 4, 5, 6}};
  std::vector<std::vector<double>> interpolated_data = TimeseriesInterpolator().getInterpolatedTimeseries(input_timestamps, data, desired_timestamps);
  std::cout << "interpolated_data.size() = " << interpolated_data.size() << std::endl;
  for (size_t i = 0; i < interpolated_data.size(); i++) {
        for (size_t j = 0; j < interpolated_data.at(i).size(); j++) {
            std::cout << "Interpolated data is " << interpolated_data.at(i).at(j) << std::endl;
        }
  }
  

  EXPECT_TRUE(result);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  srand(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

  return RUN_ALL_TESTS();
}
