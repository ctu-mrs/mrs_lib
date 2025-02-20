cmake_minimum_required(VERSION 3.12)
project(mrs_lib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
# add_definitions("-O0")
# add_definitions("-g")

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
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

find_package(yaml-cpp REQUIRED)

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

include_directories(
  include
  # ${rclcpp_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

set(LIBRARIES
  MrsLib_Geometry
  MrsLib_AttitudeConverter
  MrsLib_TimeoutManager
  MrsLib_TimerHandler
  MrsLib_ParamProvider
  MrsLib_ParamLoader
  MrsLib_ServiceClientHandler
  MrsLib_Math
  MrsLib_ScopeTimer
  MrsLib_TransformBroadcaster
  MrsLib_Profiler
  MrsLib_MsgExtractor
  MrsLib_Transformer
  MrsLib_MedianFilter
  )

## Geometry

add_library(MrsLib_Geometry SHARED
  src/geometry/misc.cpp
  src/geometry/cyclic.cpp
  src/geometry/shapes.cpp
  src/geometry/conversions.cpp
  )

target_link_libraries(MrsLib_Geometry
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_Geometry
  rclcpp
  geometry_msgs
  ${Eigen_LIBRARIES}
  )

# Utils

add_library(MrsLib_Utils SHARED
  src/utils/utils.cpp
  )

target_link_libraries(MrsLib_Utils
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_Utils
  )

# Attitude converter

add_library(MrsLib_AttitudeConverter SHARED
  src/attitude_converter/attitude_converter.cpp
  )

target_link_libraries(MrsLib_AttitudeConverter
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_AttitudeConverter
  tf2_ros
  tf2_geometry_msgs
  )

# Timeout manager

add_library(MrsLib_TimeoutManager SHARED
  src/timeout_manager/timeout_manager.cpp
  )

target_link_libraries(MrsLib_TimeoutManager
  MrsLib_Utils
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_TimeoutManager
  rclcpp
  )

# Timer handler

add_library(MrsLib_TimerHandler SHARED
  src/timer_handler/timer_handler.cpp
  )

target_link_libraries(MrsLib_TimerHandler
  MrsLib_Utils
  ${Eigen_LIBRARIES}
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
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_ParamProvider
  rclcpp
  )

# Param loader

add_library(MrsLib_ParamLoader SHARED
  src/param_loader/param_loader.cpp
  )

target_link_libraries(MrsLib_ParamLoader
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_ParamLoader
  rclcpp
  std_msgs
  )

# service client handler

add_library(MrsLib_ServiceClientHandler SHARED
  src/timer_handler/timer_handler.cpp
  )

target_link_libraries(MrsLib_ServiceClientHandler
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_ServiceClientHandler
  rclcpp
  )

# scope timer

add_library(MrsLib_ScopeTimer SHARED
  src/scope_timer/scope_timer.cpp
  )

target_link_libraries(MrsLib_ScopeTimer
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_ScopeTimer
  rclcpp
  )

# math

add_library(MrsLib_Math SHARED
  src/math/math.cpp
  )

target_link_libraries(MrsLib_Math
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_Math
  rclcpp
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
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_MsgExtractor
  rclcpp
  mrs_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  )

# transformer

add_library(MrsLib_Transformer SHARED
  src/transformer/transformer.cpp
  )

target_link_libraries(MrsLib_Transformer
  MrsLib_Geometry
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsLib_Transformer
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  mrs_msgs
  tf2_eigen
  # pcl_ros
  # pcl_conversions
  # pcl_msgs
  ${Eigen_LIBRARIES}
  )

# Median filter

add_library(MrsLib_MedianFilter SHARED
  src/median_filter/median_filter.cpp
  )

target_link_libraries(MrsLib_MedianFilter
  )

ament_target_dependencies(MrsLib_MedianFilter
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(BUILD_TESTING)

  message(WARNING "Testing enabled.")

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(TARGETS
  ${LIBRARIES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION .
)

ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
