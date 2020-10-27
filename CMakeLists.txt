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
  tf2
  tf2_geometry_msgs
  cv_bridge
  image_transport
  # dynamic_reconfigure
  )

# generate_dynamic_reconfigure_options(
#   cfg/batch_visualizer.cfg
# )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# please, NEVER commit those alternative flags with specific overrides of optimization

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -O3 -g -Wall")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -O3 -g -Wall")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -g -O0 -Wall")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

#include OpenCV
find_package(OpenCV REQUIRED HINTS /usr/local/lib)
###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_msgs
  DEPENDS Eigen
  LIBRARIES MrsLibSafetyZone MrsLibProfiler MrsLibUtils MrsLibParamLoader MrsLibMedianFilter MrsLibRepredictor MrsLibGeometryUtils MrsLibBatchVisualizer MrsLibAttitudeConverter MrsLibTransformer MrsLibImagePublisher
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
  ${OpenCV_INCLUDE_DIRS}
  # ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_library(MrsLibGeometryUtils src/geometry_utils/geometry_utils.cpp)
target_link_libraries(MrsLibGeometryUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

# add_executable(geometry_tests src/geometry_utils/tests.cpp)
# target_link_libraries(geometry_tests
#   MrsLibGeometryUtils
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   )

add_executable(cyclic_example src/geometry_utils/cyclic_example.cpp)
target_link_libraries(cyclic_example
  MrsLibGeometryUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
