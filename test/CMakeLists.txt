find_package(ament_cmake_gtest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()

link_libraries(mrs_lib::mrs_lib)

add_subdirectory(./geometry)

add_subdirectory(./utils)

add_subdirectory(./attitude_converter)

add_subdirectory(./publisher_handler)

add_subdirectory(./subscriber_handler)

add_subdirectory(./timeout_manager)

add_subdirectory(./timer_handler)

add_subdirectory(./service_client_handler)

add_subdirectory(./service_server_handler)

add_subdirectory(./math)

add_subdirectory(./lockable)

add_subdirectory(./param_loader)

add_subdirectory(./param_provider)

add_subdirectory(./dynparam_mgr)

add_subdirectory(./mutex)

add_subdirectory(./scope_timer)

add_subdirectory(./lkf)

add_subdirectory(./ukf)

add_subdirectory(./nckf)

add_subdirectory(./transform_broadcaster)

add_subdirectory(./profiler)

add_subdirectory(./msg_extractor)

add_subdirectory(./transformer)

add_subdirectory(./repredictor)

add_subdirectory(./median_filter)

add_subdirectory(./safety_zone)

add_subdirectory(./batch_visualizer)
