get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

ament_add_gtest(test_${TEST_NAME}
  test.cpp
  TIMEOUT 60
  )

target_link_libraries(test_${TEST_NAME}
  MrsLib_Geometry
  MrsLib_BatchVisualizer
  )

ament_target_dependencies(test_${TEST_NAME}
  rclcpp
  std_msgs
  mrs_msgs
  visualization_msgs
  )