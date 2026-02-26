get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

ament_add_gtest(test_${TEST_NAME}
  test.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}
  mrs_lib::mrs_lib
  rclcpp::rclcpp
  Eigen3::Eigen
  ${std_msgs_TARGETS}
)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")

# file(COPY test_config.yaml DESTINATION /tmp/mrs_lib/test/param_loader/)
# file(COPY test_config2.yaml DESTINATION /tmp/mrs_lib/test/param_loader/)
