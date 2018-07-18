cmake_minimum_required(VERSION 2.8.3)
project(mrs_lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  mrs_msgs
  tf
  )

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-diagnostics-color")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})
###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_msgs
  DEPENDS Eigen
  LIBRARIES Lkf Ukf ConvexPolygon Profiler
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_library(Lkf src/lkf/lkf.cpp)
target_link_libraries(Lkf
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(Ukf src/ukf/ukf.cpp)
target_link_libraries(Ukf
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(ConvexPolygon src/convex_polygon/convex_polygon.cpp)
target_link_libraries(ConvexPolygon
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(Profiler src/profiler/profiler.cpp)
target_link_libraries(Profiler
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
