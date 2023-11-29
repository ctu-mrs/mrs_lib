find_package(rostest REQUIRED)

# TimeoutManager

add_rostest_gtest(TimeoutManagerTest
  timeout_manager/test.cpp
  )

target_link_libraries(TimeoutManagerTest
  MrsLib_TimeoutManager
  MrsLib_Utils
  ${catkin_LIBRARIES}
  )

add_dependencies(TimeoutManagerTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# MathTest

add_rostest_gtest(MathTest
  math/test.cpp
  )

target_link_libraries(MathTest
  MrsLib_Math
  ${catkin_LIBRARIES}
  )

add_dependencies(MathTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# GeometryTest

add_rostest_gtest(GeometryTest
  geometry/test.cpp
  )

target_link_libraries(GeometryTest
  MrsLib_Geometry
  ${catkin_LIBRARIES}
  )

add_dependencies(GeometryTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# AttitudeConverterTest

add_rostest_gtest(AttitudeConverterTest
  attitude_converter/test.cpp
  )

target_link_libraries(AttitudeConverterTest
  MrsLib_AttitudeConverter
  ${catkin_LIBRARIES}
  )

add_dependencies(AttitudeConverterTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# UtilsTest

add_rostest_gtest(UtilsTest
  utils/test.cpp
  )

target_link_libraries(UtilsTest
  MrsLib_Utils
  ${catkin_LIBRARIES}
  )

add_dependencies(UtilsTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# UkfTest

add_rostest_gtest(UkfTest
  ukf/test.cpp
  )

target_link_libraries(UkfTest
  ${catkin_LIBRARIES}
  )

add_dependencies(UkfTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# TransformerTest

add_rostest_gtest(TransformerTest
  transformer/test.cpp
  )

target_link_libraries(TransformerTest
  MrsLib_Transformer
  MrsLib_Geometry
  ${catkin_LIBRARIES}
  )

add_dependencies(TransformerTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# ParamLoaderTest

add_rostest_gtest(ParamLoaderTest
  param_loader/test.cpp
  )

target_link_libraries(ParamLoaderTest
  MrsLib_ParamLoader
  ${catkin_LIBRARIES}
  )

add_dependencies(ParamLoaderTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# SubscribeHandlerTest

add_rostest_gtest(SubscribeHandlerTest
  subscribe_handler/test.cpp
  )

target_link_libraries(SubscribeHandlerTest
  MrsLib_TimeoutManager
  ${catkin_LIBRARIES}
  )

add_dependencies(SubscribeHandlerTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# RepredictorTest

add_rostest_gtest(RepredictorTest
  repredictor/test.cpp
  )

target_link_libraries(RepredictorTest
  ${catkin_LIBRARIES}
  )

add_dependencies(RepredictorTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# TimerTest

add_rostest_gtest(TimerTest
  timer/test.cpp
  )

target_link_libraries(TimerTest
  MrsLib_Timer
  MrsLib_Utils
  ${catkin_LIBRARIES}
  )

add_dependencies(TimerTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# MedianFilterTest

add_rostest_gtest(MedianFilterTest
  median_filter/test.cpp
  )

target_link_libraries(MedianFilterTest
  MrsLib_MedianFilter
  ${catkin_LIBRARIES}
  )

add_dependencies(MedianFilterTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# ServiceClientHandlerTest

add_rostest_gtest(ServiceClientHandlerTest
  service_client_handler/test.cpp
  )

target_link_libraries(ServiceClientHandlerTest
  ${catkin_LIBRARIES}
  )

add_dependencies(ServiceClientHandlerTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# PublisherHandlerTest

add_rostest_gtest(PublisherHandlerTest
  publisher_handler/test.cpp
  )

target_link_libraries(PublisherHandlerTest
  ${catkin_LIBRARIES}
  )

add_dependencies(PublisherHandlerTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# add_rostest_gtest(VectorConverterTest
#   vector_converter/test.cpp
#   )
#
# target_link_libraries(VectorConverterTest
#   ${catkin_LIBRARIES}
#   )
#
# target_include_directories(VectorConverterTest PRIVATE
#   ${pcl_ros_INCLUDE_DIRS}
#   )
#
# add_dependencies(VectorConverterTest
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   )
