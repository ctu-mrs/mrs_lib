find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

add_subdirectory(./geometry)

add_subdirectory(./utils)

add_subdirectory(./attitude_converter)

add_subdirectory(./publisher_handler)

add_subdirectory(./timeout_manager)

add_subdirectory(./timer_handler)

add_subdirectory(./service_client_handler)

add_subdirectory(./math)

add_subdirectory(./param_loader)

add_subdirectory(./mutex)

add_subdirectory(./scope_timer)

add_subdirectory(./lkf)

add_subdirectory(./ukf)

add_subdirectory(./dkf)

add_subdirectory(./transform_broadcaster)

add_subdirectory(./profiler)

# add_subdirectory(./msg_extractor)
