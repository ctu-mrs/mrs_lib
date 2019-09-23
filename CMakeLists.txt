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

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -g -O0 -Wall")

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
  LIBRARIES Lkf Ukf ConvexPolygon Profiler Utils ParamLoader MedianFilter Repredictor geometry_utils
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

add_library(Repredictor src/repredictor/Repredictor.cpp src/repredictor/MRS_odom_repredictor.cpp)
target_link_libraries(Repredictor
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
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

add_executable(lkf_example src/lkf/example.cpp)
target_link_libraries(lkf_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(ukf_tests src/ukf/ukf_tests.cpp src/ukf/ukf.cpp)
target_link_libraries(ukf_tests
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(ukf_example src/ukf/example.cpp)
target_link_libraries(ukf_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(nckf_tests src/nckf/nckf_tests.cpp)
target_link_libraries(nckf_tests
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

add_library(Utils src/utils/utils.cpp)
target_link_libraries(Utils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(ParamLoader src/param_loader/ParamLoader.cpp)
target_link_libraries(ParamLoader
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(param_loader_tests src/param_loader/param_loader_tests.cpp)
target_link_libraries(param_loader_tests
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(param_loader_example src/param_loader/example.cpp)
target_link_libraries(param_loader_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(subscribe_handler_tests src/subscribe_handler/subscribe_handler_tests.cpp)
target_link_libraries(subscribe_handler_tests
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(subscribe_handler_example src/subscribe_handler/example.cpp)
target_link_libraries(subscribe_handler_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(rheiv_example src/rheiv/example.cpp)
target_link_libraries(rheiv_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MedianFilter src/median_filter/MedianFilter.cpp)
target_link_libraries(MedianFilter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(geometry_utils src/geometry_utils/geometry_utils.cpp)
target_link_libraries(geometry_utils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_executable(geometry_utils_tests src/geometry_utils/geometry_utils.cpp src/geometry_utils/geometry_utils_tests.cpp)
target_link_libraries(geometry_utils_tests
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )
