cmake_minimum_required(VERSION 3.12)
project(mrs_lib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(Threads REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

find_package(yaml-cpp REQUIRED)

include_directories(
  include
  # ${rclcpp_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  yaml-cpp
)

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
  MrsLib_BatchVisualizer 
  )

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
  ${Eigen_LIBRARIES}
  )

# Utils

add_library(MrsLib_Utils SHARED
  src/utils/utils.cpp
  )

ament_target_dependencies(MrsLib_Utils
  ${Eigen_LIBRARIES}
  )

# Attitude converter

add_library(MrsLib_AttitudeConverter SHARED
  src/attitude_converter/attitude_converter.cpp
  )

ament_target_dependencies(MrsLib_AttitudeConverter
  tf2_ros
  tf2_geometry_msgs
  ${Eigen_LIBRARIES}
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
  ${Eigen_LIBRARIES}
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
  ${Eigen_LIBRARIES}
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
  ${Eigen_LIBRARIES}
  )

# Param loader

add_library(MrsLib_ParamLoader SHARED
  src/param_loader/param_loader.cpp
  )

ament_target_dependencies(MrsLib_ParamLoader
  rclcpp
  std_msgs
  ${Eigen_LIBRARIES}
  )

# service client handler

add_library(MrsLib_ServiceClientHandler SHARED
  src/timer_handler/timer_handler.cpp
  )

target_link_libraries(MrsLib_ServiceClientHandler
  )

ament_target_dependencies(MrsLib_ServiceClientHandler
  rclcpp
  ${Eigen_LIBRARIES}
  )

# scope timer

add_library(MrsLib_ScopeTimer SHARED
  src/scope_timer/scope_timer.cpp
  )

target_link_libraries(MrsLib_ScopeTimer
  )

ament_target_dependencies(MrsLib_ScopeTimer
  rclcpp
  ${Eigen_LIBRARIES}
  )

# math

add_library(MrsLib_Math SHARED
  src/math/math.cpp
  )

target_link_libraries(MrsLib_Math
  )

ament_target_dependencies(MrsLib_Math
  rclcpp
  ${Eigen_LIBRARIES}
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
  ${Eigen_LIBRARIES}
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
  ${Eigen_LIBRARIES}
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
  ${Eigen_LIBRARIES}
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
  MrsLib_Geometry
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
