get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

ament_add_gtest(test_${TEST_NAME}
  test.cpp
  TIMEOUT 60
)

target_link_libraries(test_${TEST_NAME}
  mrs_lib::mrs_lib
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

target_compile_definitions(test_${TEST_NAME} PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})
