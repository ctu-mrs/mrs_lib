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

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -O3 -Wall")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -O0 -g -Wall")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -O3 -g -Wall")
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
  LIBRARIES MrsLibSafetyZone MrsLibProfiler MrsLibUtils MrsLibParamLoader MrsLibMedianFilter MrsLibRepredictor MrsLibGeometryUtils MrsLibBatchVisualizer MrsLibAttitudeConverter MrsLibTransformer MrsLibImagePublisher MrsLibLkfLegacy
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

add_library(MrsLibRepredictor src/repredictor/repredictor.cpp src/repredictor/MRS_odom_repredictor.cpp)
target_link_libraries(MrsLibRepredictor
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibLkfLegacy src/lkf/lkf_legacy.cpp)
target_link_libraries(MrsLibLkfLegacy
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(lkf_example src/lkf/example.cpp)
target_link_libraries(lkf_example
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

add_library(MrsLibSafetyZone src/safety_zone/safety_zone.cpp
  src/safety_zone/point_obstacle/point_obstacle.cpp
  src/safety_zone/line_operations.cpp
  src/safety_zone/polygon/polygon.cpp
  )

target_link_libraries(MrsLibSafetyZone
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibProfiler src/profiler/profiler.cpp)
target_link_libraries(MrsLibProfiler
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(mutex_tests src/mutex/mutex_tests.cpp)
target_link_libraries(mutex_tests
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibAttitudeConverter src/attitude_converter/attitude_converter.cpp)
target_link_libraries(MrsLibAttitudeConverter
  MrsLibGeometryUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibTransformer src/transformer/transformer.cpp)
target_link_libraries(MrsLibTransformer
  MrsLibAttitudeConverter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibUtils src/utils/utils.cpp)
target_link_libraries(MrsLibUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibParamLoader src/param_loader/param_loader.cpp)
target_link_libraries(MrsLibParamLoader
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(param_loader_example src/param_loader/example.cpp)
target_link_libraries(param_loader_example
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

add_library(MrsLibMedianFilter src/median_filter/median_filter.cpp)
target_link_libraries(MrsLibMedianFilter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibGeometryUtils src/geometry_utils/geometry_utils.cpp)
target_link_libraries(MrsLibGeometryUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(MrsLibBatchVisualizer src/batch_visualizer/batch_visualizer.cpp)
target_link_libraries(MrsLibBatchVisualizer
  MrsLibGeometryUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(MrsLibImagePublisher src/image_publisher/image_publisher.cpp)
target_link_libraries(MrsLibImagePublisher
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_executable(batch_visualizer_demo src/batch_visualizer/batch_visualizer_demo.cpp)
target_link_libraries(batch_visualizer_demo
  MrsLibGeometryUtils
  MrsLibBatchVisualizer
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(
    GeometryUtilsTest
    test/geometry_utils/geometry_utils.test
    test/geometry_utils/test.cpp
  )

  target_link_libraries(
    GeometryUtilsTest
    MrsLibGeometryUtils
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    GeometryUtilsTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    AttitudeConverterTest
    test/attitude_converter/attitude_converter.test
    test/attitude_converter/test.cpp
  )

  target_link_libraries(
    AttitudeConverterTest
    MrsLibAttitudeConverter
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    AttitudeConverterTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    UtilsTest
    test/utils/utils.test
    test/utils/test.cpp
  )

  target_link_libraries(
    UtilsTest
    MrsLibUtils
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    UtilsTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    UkfTest
    test/ukf/ukf.test
    test/ukf/test.cpp
  )

  target_link_libraries(
    UkfTest
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    UkfTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    TransformerTest
    test/transformer/transformer.test
    test/transformer/test.cpp
  )

  target_link_libraries(
    TransformerTest
    MrsLibTransformer
    MrsLibAttitudeConverter
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    TransformerTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    ParamLoaderTest
    test/param_loader/param_loader.test
    test/param_loader/test.cpp
  )

  target_link_libraries(
    ParamLoaderTest
    MrsLibParamLoader
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    ParamLoaderTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    SubscribeHandlerTest
    test/subscribe_handler/subscribe_handler.test
    test/subscribe_handler/test.cpp
  )

  target_link_libraries(
    SubscribeHandlerTest
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    SubscribeHandlerTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

endif()
