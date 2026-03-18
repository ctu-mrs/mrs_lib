cmake_minimum_required(VERSION 3.28)
project(mrs_lib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_lib]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

find_package(ament_cmake_python REQUIRED)
find_package(mrs_cmake REQUIRED)

set(DEPENDENCIES
  geometry_msgs
  mrs_msgs
  nav_msgs
  # pcl_conversions
  # pcl_msgs
  # pcl_ros
  rclcpp
  rclcpp_components
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  yaml-cpp
  visualization_msgs
  Eigen3
  yaml-cpp
  Boost
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

add_library(mrs_lib_mrs_lib SHARED)
add_library(mrs_lib::mrs_lib ALIAS mrs_lib_mrs_lib)
set_target_properties(mrs_lib_mrs_lib
  PROPERTIES
    EXPORT_NAME mrs_lib
    OUTPUT_NAME mrs_lib
)

target_sources(mrs_lib_mrs_lib
  PRIVATE
    # src/attitude_converter
    "src/attitude_converter/attitude_converter.cpp"
    # src/batch_visualizer
    "src/batch_visualizer/batch_visualizer.cpp"
    "src/batch_visualizer/visual_object.cpp"
    # src/coro/internal
    "src/coro/internal/thread_local_continuation_scheduler.cpp"
    # src/dynparam_mgr
    "src/dynparam_mgr/dynparam_mgr.cpp"
    # src/errorgraph
    "src/errorgraph/errorgraph.cpp"
    "src/errorgraph/error_publisher.cpp"
    # src/geometry
    "src/geometry/conversions.cpp"
    "src/geometry/cyclic.cpp"
    "src/geometry/misc.cpp"
    "src/geometry/shapes.cpp"
    # src/iir_filter
    "src/iir_filter/iir_filter.cpp"
    # src/math
    "src/math/math.cpp"
    # src/median_filter
    "src/median_filter/median_filter.cpp"
    # src/msg_extractor
    "src/msg_extractor/msg_extractor.cpp"
    # src/notch_filter
    "src/notch_filter/notch_filter.cpp"
    # src/param_loader
    "src/param_loader/param_loader.cpp"
    # src/param_provider
    "src/param_provider/param_provider.cpp"
    # src/profiler
    "src/profiler/profiler.cpp"
    # src/safety_zone
    "src/safety_zone/line_operations.cpp"
    "src/safety_zone/polygon.cpp"
    "src/safety_zone/prism.cpp"
    "src/safety_zone/safety_zone.cpp"
    "src/safety_zone/static_edges_visualization.cpp"
    # src/scope_timer
    "src/scope_timer/scope_timer.cpp"
    # src/timeout_manager
    "src/timeout_manager/timeout_manager.cpp"
    # src/timer_handler
    "src/timer_handler/timer_handler.cpp"
    # src/transform_broadcaster
    "src/transform_broadcaster/transform_broadcaster.cpp"
    # src/transformer
    "src/transformer/transformer.cpp"
    # src/utils
    "src/utils/utils.cpp"
  PUBLIC FILE_SET HEADERS BASE_DIRS "include"
  FILES
    # include/mrs_lib
    "include/mrs_lib/attitude_converter.h"
    "include/mrs_lib/batch_visualizer.h"
    "include/mrs_lib/dkf.h"
    "include/mrs_lib/dynparam_mgr.h"
    "include/mrs_lib/gps_conversions.h"
    "include/mrs_lib/iir_filter.h"
    "include/mrs_lib/integration_test.h"
    "include/mrs_lib/kalman_filter.h"
    "include/mrs_lib/lkf.h"
    "include/mrs_lib/lockable.h"
    "include/mrs_lib/math.h"
    "include/mrs_lib/median_filter.h"
    "include/mrs_lib/mrs_lib.h"
    "include/mrs_lib/msg_extractor.h"
    "include/mrs_lib/mutex.h"
    "include/mrs_lib/nckf.h"
    "include/mrs_lib/node.h"
    "include/mrs_lib/notch_filter.h"
    "include/mrs_lib/param_loader.h"
    "include/mrs_lib/param_provider.h"
    "include/mrs_lib/profiler.h"
    "include/mrs_lib/publisher_handler.h"
    "include/mrs_lib/quadratic_throttle_model.h"
    "include/mrs_lib/repredictor.h"
    "include/mrs_lib/safety_zone.h"
    "include/mrs_lib/scope_timer.h"
    "include/mrs_lib/service_client_handler.h"
    "include/mrs_lib/service_server_handler.h"
    "include/mrs_lib/subscriber_handler.h"
    "include/mrs_lib/timeout_manager.h"
    "include/mrs_lib/timer_handler.h"
    "include/mrs_lib/transform_broadcaster.h"
    "include/mrs_lib/transformer.h"
    "include/mrs_lib/ukf.h"
    "include/mrs_lib/utils.h"
    "include/mrs_lib/visual_object.h"
    # include/mrs_lib/coro
    "include/mrs_lib/coro/runners.hpp"
    "include/mrs_lib/coro/task.hpp"
    "include/mrs_lib/coro/task.impl.hpp"
    # include/mrs_lib/coro/internal
    "include/mrs_lib/coro/internal/attributes.hpp"
    "include/mrs_lib/coro/internal/thread_local_continuation_scheduler.hpp"
    # include/mrs_lib/errorgraph
    "include/mrs_lib/errorgraph/node_id.h"
    "include/mrs_lib/errorgraph/errorgraph.h"
    "include/mrs_lib/errorgraph/error_publisher.h"
    # include/mrs_lib/geometry
    "include/mrs_lib/geometry/conversions_eigen.h"
    "include/mrs_lib/geometry/conversions.h"
    "include/mrs_lib/geometry/cyclic.h"
    "include/mrs_lib/geometry/misc.h"
    "include/mrs_lib/geometry/shapes.h"
    # include/mrs_lib/impl
    "include/mrs_lib/impl/dynparam_mgr.hpp"
    "include/mrs_lib/impl/param_loader.hpp"
    "include/mrs_lib/impl/param_provider.hpp"
    "include/mrs_lib/impl/publisher_handler.hpp"
    "include/mrs_lib/impl/service_client_handler.hpp"
    "include/mrs_lib/impl/service_server_handler.hpp"
    "include/mrs_lib/impl/subscriber_handler.hpp"
    "include/mrs_lib/impl/timer_handler.hpp"
    "include/mrs_lib/impl/transformer.hpp"
    "include/mrs_lib/impl/ukf.hpp"
    # include/mrs_lib/internal
    "include/mrs_lib/internal/coroutine_callback_helpers.hpp"
    # include/mrs_lib/safety_zone
    "include/mrs_lib/safety_zone/line_operations.h"
    "include/mrs_lib/safety_zone/polygon.h"
    "include/mrs_lib/safety_zone/prism.h"
    "include/mrs_lib/safety_zone/static_edges_visualization.h"
)

target_link_libraries(mrs_lib_mrs_lib
  PUBLIC
    rclcpp::rclcpp
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    tf2_eigen::tf2_eigen
    Boost::headers
    ${OpenCV_LIBRARIES}
    # msgs
    ${geometry_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${mrs_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
)

# Dynamic param manager tests

add_executable(dynparam_tests
  src/dynparam_mgr/dynparam_tests.cpp
)

target_link_libraries(dynparam_tests
  PRIVATE
    rclcpp::rclcpp
    mrs_lib::mrs_lib
)

## | --------------- subscribe handler examples --------------- |

# TODO why is it built from all these different files?
add_executable(subscribe_handler_example
  src/subscriber_handler/example.cpp
  src/timeout_manager/timeout_manager.cpp
  src/timer_handler/timer_handler.cpp
)

target_link_libraries(subscribe_handler_example
  PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    mrs_lib::mrs_lib
)

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  mrs_verify_interface_header_sets(mrs_lib_mrs_lib)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS
    mrs_lib_mrs_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  FILE_SET HEADERS DESTINATION "include/${PROJECT_NAME}"
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

# Kept for backward compatibility
ament_export_libraries(
  mrs_lib_mrs_lib
)

# Kept for backward compatibility
ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
