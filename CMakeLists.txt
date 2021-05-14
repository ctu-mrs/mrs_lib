cmake_minimum_required(VERSION 3.1.2)
project(mrs_lib)

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
  tf2_sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  # dynamic_reconfigure
  )

# generate_dynamic_reconfigure_options(
#   cfg/batch_visualizer.cfg
# )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# please, NEVER commit those alternative flags with specific overrides of optimization
#  add the address sanitizer compiler flags to debug build type
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined -fno-omit-frame-pointer") # because normal release is just -O2
#  add the address sanitizer linker flags to debug build type
set(CMAKE_LD_FLAGS_DEBUG  "${CMAKE_LD_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined") # because normal release is just -O2
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3") # because normal release is just -O2
# please, really do not commit your temporary custom flags :-)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

# include OpenCV
find_package(OpenCV REQUIRED HINTS /usr/local/lib)

# include PCL
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs pcl_ros mrs_msgs
  DEPENDS Eigen
  LIBRARIES MrsLibSafetyZone MrsLibProfiler MrsLibScopeTimer MrsLibIirFilter MrsLibNotchFilter MrsLibUtils MrsLibParamLoader MrsLibMedianFilter MrsLibOdomLKF MrsLibGeometry MrsLibBatchVisualizer MrsLibAttitudeConverter MrsLibTransformer MrsLibImagePublisher MrsLibTimer MrsLibTransformBroadcaster
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_executable(repredictor_example src/repredictor/example.cpp)
target_link_libraries(repredictor_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(service_client_handler_example src/service_client_handler/example.cpp)
target_link_libraries(service_client_handler_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibOdomLKF src/lkf/LKF_MRS_odom.cpp)
target_link_libraries(MrsLibOdomLKF
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(lkf_example src/lkf/example.cpp)
target_link_libraries(lkf_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# # Slows the compilation of the library waaaay too much - just generate the conversions on-demand
# # instead of pre-generating all combinations.
# add_library(MrsLibVectorConverter src/vector_converter/vector_converter.cpp)
# target_link_libraries(MrsLibVectorConverter
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
#   ${PCL_LIBRARIES}
#   )
# target_include_directories(MrsLibVectorConverter PRIVATE
#   ${PCL_INCLUDE_DIRS}
#   )

add_executable(vector_converter_example src/vector_converter/example.cpp)
target_link_libraries(vector_converter_example
  # MrsLibVectorConverter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${PCL_LIBRARIES}
  )
target_include_directories(vector_converter_example PRIVATE
  ${PCL_INCLUDE_DIRS}
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

add_library(MrsLibScopeTimer src/scope_timer/scope_timer.cpp)
target_link_libraries(MrsLibScopeTimer
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibIirFilter src/iir_filter/iir_filter.cpp)
target_link_libraries(MrsLibIirFilter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibNotchFilter src/notch_filter/notch_filter.cpp)
target_link_libraries(MrsLibNotchFilter
  MrsLibIirFilter
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
  MrsLibGeometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(MrsLibTransformer src/transformer/transformer.cpp)
target_link_libraries(MrsLibTransformer
  MrsLibAttitudeConverter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
target_include_directories(MrsLibTransformer PRIVATE
  ${PCL_INCLUDE_DIRS}
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

add_library(MrsLibTransformBroadcaster src/transform_broadcaster/transform_broadcaster.cpp)
target_link_libraries(MrsLibTransformBroadcaster
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
  MrsLibTimer
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

add_library(MrsLibBatchVisualizer src/batch_visualizer/batch_visualizer.cpp)
target_link_libraries(MrsLibBatchVisualizer
  MrsLibGeometry
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
  MrsLibGeometry
  MrsLibBatchVisualizer
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(MrsLibGeometry src/geometry/misc.cpp src/geometry/cyclic.cpp src/geometry/shapes.cpp)
target_link_libraries(MrsLibGeometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(MrsLibTimer src/timer/timer.cpp)
target_link_libraries(MrsLibTimer
  ${catkin_LIBRARIES}
  )

add_executable(geometry_tests src/geometry/tests.cpp)
target_link_libraries(geometry_tests
  MrsLibGeometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_executable(cyclic_example src/geometry/cyclic_example.cpp)
target_link_libraries(cyclic_example
  MrsLibGeometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(
    GeometryTest
    test/geometry/geometry.test
    test/geometry/test.cpp
  )

  target_link_libraries(
    GeometryTest
    MrsLibGeometry
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    GeometryTest
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
    MrsLibTimer 
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    SubscribeHandlerTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    RepredictorTest
    test/repredictor/repredictor.test
    test/repredictor/test.cpp
  )
  target_link_libraries(
    RepredictorTest
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    RepredictorTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    ServiceClientHandlerTest
    test/service_client_handler/service_client_handler.test
    test/service_client_handler/test.cpp
  )
  target_link_libraries(
    ServiceClientHandlerTest
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    ServiceClientHandlerTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  add_rostest_gtest(
    VectorConverterTest
    test/vector_converter/vector_converter.test
    test/vector_converter/test.cpp
  )
  target_link_libraries(
    VectorConverterTest
    ${catkin_LIBRARIES}
  )
  target_include_directories(VectorConverterTest PRIVATE
    ${PCL_INCLUDE_DIRS}
  )
  add_dependencies(
    VectorConverterTest
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

endif()
