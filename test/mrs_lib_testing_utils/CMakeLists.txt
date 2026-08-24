ament_find_gtest()

add_library(mrs_lib_testing_utils INTERFACE)
add_library(mrs_lib::testing_utils ALIAS mrs_lib_testing_utils)

target_sources(mrs_lib_testing_utils
  INTERFACE FILE_SET HEADERS BASE_DIRS "include"
  FILES
    "include/mrs_lib_testing/ros_fixtures.hpp"
)

target_link_libraries(mrs_lib_testing_utils
  INTERFACE
    rclcpp::rclcpp
    ${GTEST_LIBRARIES}
)

