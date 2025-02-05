cmake_minimum_required(VERSION 3.12)
project(mrs_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(Threads REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

set(DEPENDENCIES
  rclcpp
  geometry_msgs
)

set(LIBRARIES
  MrsLib_Geometry
  MrsLib_AttitudeConverter
  MrsLib_TimeoutManager
  MrsLib_TimerHandler
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
