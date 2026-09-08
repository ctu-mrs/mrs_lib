get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

add_executable(test_${TEST_NAME}
  logger_test.cpp
)

target_link_libraries(test_${TEST_NAME}
  mrs_lib::mrs_lib
  rclcpp::rclcpp
)

ament_add_test(test_${TEST_NAME}
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND
    $<TARGET_FILE:Python3::Interpreter>
    "${CMAKE_CURRENT_SOURCE_DIR}/runner.py"
    "--bin-path"
    $<TARGET_FILE:test_${TEST_NAME}>
  TIMEOUT 60
)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
