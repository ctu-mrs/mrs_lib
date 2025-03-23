cmake_minimum_required(VERSION 3.12)
project(mrs_lib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")

if(MRS_ENABLE_TESTING)
  message(WARNING "Setting optimizaton levels for tests")
  add_definitions("-O0")
  add_definitions("-g")
else()
  add_definitions("-O2")
endif()

if(COVERAGE)
  message(WARNING "building with --coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

find_package(ament_cmake REQUIRED)

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
)

set(LIBRARIES
  MrsLib_Geometry
  MrsLib_AttitudeConverter
  MrsLib_TimeoutManager
  MrsLib_TimerHandler
  MrsLib_ParamProvider
  MrsLib_ParamLoader
  MrsLib_Math
  MrsLib_ScopeTimer
  MrsLib_TransformBroadcaster
  MrsLib_Profiler
  MrsLib_MsgExtractor
  MrsLib_BatchVisualizer
  MrsLib_Transformer
  MrsLib_MedianFilter
  MrsLib_SafetyZone
  MrsLib_IIRFilter
  MrsLib_Utils
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

## Geometry

add_library(MrsLib_Geometry SHARED
  src/geometry/misc.cpp
  src/geometry/cyclic.cpp
  src/geometry/shapes.cpp
  src/geometry/conversions.cpp
  )

ament_target_dependencies(MrsLib_Geometry
  rclcpp
  geometry_msgs
  Eigen3
  )

# Utils

add_library(MrsLib_Utils SHARED
  src/utils/utils.cpp
  )

ament_target_dependencies(MrsLib_Utils
  Eigen3
  )

# Attitude converter

add_library(MrsLib_AttitudeConverter SHARED
  src/attitude_converter/attitude_converter.cpp
  )

ament_target_dependencies(MrsLib_AttitudeConverter
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  )

# Timeout manager

add_library(MrsLib_TimeoutManager SHARED
  src/timeout_manager/timeout_manager.cpp
  )

target_link_libraries(MrsLib_TimeoutManager
  MrsLib_Utils
  )

ament_target_dependencies(MrsLib_TimeoutManager
  rclcpp
  Eigen3
  )

# Timer handler

add_library(MrsLib_TimerHandler SHARED
  src/timer_handler/timer_handler.cpp
  )

target_link_libraries(MrsLib_TimerHandler
  MrsLib_Utils
  )

ament_target_dependencies(MrsLib_TimerHandler
  rclcpp
  )

# Param provider

add_library(MrsLib_ParamProvider SHARED
  src/param_loader/param_provider.cpp
  )

# yaml-cpp does not use standart naming conventions which create issues with ament_target_dependencies()
target_link_libraries(MrsLib_ParamProvider
  yaml-cpp
  )

ament_target_dependencies(MrsLib_ParamProvider
  rclcpp
  Eigen3
  )

# Param loader

add_library(MrsLib_ParamLoader SHARED
  src/param_loader/param_loader.cpp
  )

ament_target_dependencies(MrsLib_ParamLoader
  rclcpp
  std_msgs
  Eigen3
  )

# scope timer

add_library(MrsLib_ScopeTimer SHARED
  src/scope_timer/scope_timer.cpp
  )

ament_target_dependencies(MrsLib_ScopeTimer
  rclcpp
  )

# math

add_library(MrsLib_Math SHARED
  src/math/math.cpp
  )

ament_target_dependencies(MrsLib_Math
  rclcpp
  Eigen3
  )

# transform broadcaster

add_library(MrsLib_TransformBroadcaster SHARED
  src/transform_broadcaster/transform_broadcaster.cpp
  )

target_link_libraries(MrsLib_TransformBroadcaster
  )

ament_target_dependencies(MrsLib_TransformBroadcaster
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  )

# profiler

add_library(MrsLib_Profiler SHARED
  src/profiler/profiler.cpp
  )

target_link_libraries(MrsLib_Profiler
  )

ament_target_dependencies(MrsLib_Profiler
  rclcpp
  mrs_msgs
  )

# msg_extractor

add_library(MrsLib_MsgExtractor SHARED
  src/msg_extractor/msg_extractor.cpp
  )

target_link_libraries(MrsLib_MsgExtractor
  MrsLib_AttitudeConverter
  )

ament_target_dependencies(MrsLib_MsgExtractor
  rclcpp
  mrs_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  Eigen3
  )

# transformer

add_library(MrsLib_Transformer SHARED
  src/transformer/transformer.cpp
  )

target_link_libraries(MrsLib_Transformer
  MrsLib_Geometry
  ${OpenCV_LIBRARIES}
  )

ament_target_dependencies(MrsLib_Transformer
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  mrs_msgs
  tf2_eigen
  Eigen3
  # pcl_ros
  # pcl_conversions
  # pcl_msgs
  )

# Median filter

add_library(MrsLib_MedianFilter SHARED
  src/median_filter/median_filter.cpp
  )

ament_target_dependencies(MrsLib_MedianFilter
  )

# safety zone

add_library(MrsLib_SafetyZone
  src/safety_zone/safety_zone.cpp
  src/safety_zone/line_operations.cpp
  src/safety_zone/polygon/polygon.cpp
  )

ament_target_dependencies(MrsLib_SafetyZone
  rclcpp
  geometry_msgs
  Eigen3
  )

# IIR filter

add_library(MrsLib_IIRFilter
  src/iir_filter/iir_filter.cpp
  )

ament_target_dependencies(MrsLib_IIRFilter
  Eigen3
  )

# batch_visualizer

add_library(MrsLib_BatchVisualizer SHARED
  src/batch_visualizer/visual_object.cpp
  src/batch_visualizer/batch_visualizer.cpp
  )

target_link_libraries(MrsLib_BatchVisualizer
  MrsLib_Geometry
  )

ament_target_dependencies(MrsLib_BatchVisualizer
  rclcpp
  mrs_msgs
  visualization_msgs
  Eigen3
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(BUILD_TESTING)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  DIRECTORY include
  DESTINATION .
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# use both becasue the downstream library may use "classic"/"modern" cmake

# "modern" cmake approach to export pkgs
# ament_export_targets(
#   export_${PROJECT_NAME} HAS_LIBRARY_TARGET
# )

ament_export_include_directories(
  include
)

# "classic" cmake approach to export pkgs
ament_export_libraries(
  ${LIBRARIES}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
